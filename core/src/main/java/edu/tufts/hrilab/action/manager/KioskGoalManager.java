package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.dialogue.Question;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.nio.file.Paths;
import java.util.*;
import java.util.concurrent.TimeUnit;

//Temporary Solution - not sure where this should live or how it should be implemented in the end.
//TODO: currently only implemented with the QueueEM in mind
//TODO: Extend this class to allow for a kiosk-goal to be defined and active per-agent/team.
public class KioskGoalManager extends DiarcComponent {

  /**
   * Experimental - Kiosk Goal implementation in external component from the EM. Uses listeners for lifecycle
   * Other options:
   *  Specific persistent actions can be written in ways to mimic kiosk mode (utilizing low priority, internal
   *    interruptible sleeps in implementation, and onResume triggering recovery with postrecovery restarting the action)
   *  Push into EM itself
   *  Push into agentTeams or other external component using goalStatus listeners to manage kiosk goal lifecycle
   *
   *  Kiosk Mode:
   *    When a goal has not been submitted
   */
  private static final Object kioskGoalAccessLock = new Object();
  private boolean kioskModeEnabled;
  private Predicate kioskGoalPredicate = Factory.createPredicate("chargeAtHome","ava");
  private long kioskTimeout = 2700; //seconds
  private TimerTask kioskTimerTask;
  private Timer kioskTimer;
  private long kioskGoalGid;
  private Set<Long> currentGids;
  private Set<Symbol> agentsWithQuestions;
  private String kioskSettingsFile = null;

  public KioskGoalManager() {
    super();
  };

  @Override
  protected void init() {
    kioskTimer = null;
    kioskTimerTask = null;
    kioskGoalGid = -1;
    currentGids = new HashSet<>();
    agentsWithQuestions = new HashSet<>();
    if (kioskSettingsFile != null) {
      loadKioskSettings(kioskSettingsFile);
    }
  }

  //TODO: Change so this isn't required to be instantiated after dialogue and GM
  @Override
  protected void postRegistrationInit() {
    //getAgentList()
    registerDialogueListener();
    registerGoalStatusListener();
    registerQuestionAskingListener();

    if (kioskModeEnabled) {
      resetKioskTimer();
    }
  }


  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("kioskSettingsFile").hasArg().argName("file").desc("File containing kiosk configuration to load from on startup and write to on shutdown").build());
    options.add(Option.builder("enabled").desc("whether kiosk mode should start enabled").build());
    options.add(Option.builder("goal").hasArg().desc("kiosk goal predicate").build());
    options.add(Option.builder("timeout").hasArg().desc("timeout for kiosk mode (seconds)").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("enabled")) {
      kioskModeEnabled = true;
    }
    if (cmdLine.hasOption("goal")) {
      kioskGoalPredicate = Factory.createPredicate(cmdLine.getOptionValue("goal"));
    }
    if (cmdLine.hasOption("timeout")) {
      kioskTimeout = Integer.parseInt(cmdLine.getOptionValue("timeout"));
    }
    if (cmdLine.hasOption("kioskSettingsFile")) {
      String filename = cmdLine.getOptionValue("kioskSettingsFile");
      String homeDir = System.getProperty("user.home");
      String kioskSettingsDir = Paths.get(homeDir, ".diarc","KioskConfigurations").toString();
      kioskSettingsFile = kioskSettingsDir + File.separator + filename;
    }
  }

  //private void getAgentList() {}

  private void registerDialogueListener() {
    TRADEServiceInfo tsi = getMyService("dialogueHistoryCallback", Utterance.class, Boolean.class);
    if (tsi == null) {
      log.error("[registerDialogueListener] unable to get dialogueHistoryCallback tsi");
      return;
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForDialogueHistoryNotifications").argTypes(TRADEServiceInfo.class)).call(void.class, tsi);
    } catch (TRADEException e) {
      log.error("[registerDialogueListener] unable to register for dialogue history notifications", e);
    }
  }

  private void registerGoalStatusListener() {
    TRADEServiceInfo tsi = getMyService("goalStatusChangedCallback", GoalInfo.class);
    if (tsi == null) {
      log.error("[registerGoalStatusListener] unable to get goalStatusChangedCallback tsi");
      return;
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("registerGoalManagerNotification").argTypes(TRADEServiceInfo.class, NotificationType.class)).call(void.class, tsi, NotificationType.GOAL_STATUS_CHANGE);
    } catch (TRADEException e) {
      log.error("[registerGoalStatusListener] unable to register for goal status update notifications", e);
    }
  }

  private void registerQuestionAskingListener() {
    TRADEServiceInfo tsi = getMyService("questionAskingCallback", Symbol.class, Boolean.class, Question.class);
    if (tsi == null) {
      log.error("[registerQuestionAskingListener] unable to get questionAskingCallback tsi");
      return;
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForQuestionAskingNotifications").argTypes(TRADEServiceInfo.class)).call(void.class, tsi);
    } catch (TRADEException e) {
      log.error("[registerQuestionAskingListener] unable to register for question asking notifications", e);
    }
  }

  private void cancelKioskTimer() {
    log.debug("[cancelKioskTimer] in method");
    synchronized (kioskGoalAccessLock) {
      //does this need to worry about the kiosk task starting concurrently?
      if (kioskTimerTask != null && kioskTimer != null) {
        log.debug("[cancelKioskTimer] canceling timer and task");
        //do we need to cancel the timer or the task, or both? what's the difference?
        //  does canceling the task and then resubmitting it allow it to still be executed?
        kioskTimerTask.cancel();
        kioskTimer.cancel();
      }
    }
  }

  private void startKioskTimer() {
    log.debug("[startKioskTimer] in method");
    synchronized (kioskGoalAccessLock) {
      if (!kioskModeEnabled) {
        log.warn("[startKioskTimer] kiosk mode not enabled, not starting timer");
        return;
      } else if (!currentGids.isEmpty()) {
        log.warn("[startKioskTimer] other goals exist, not starting timer");
        return;
      } else if (kioskGoalGid != -1) {
        log.warn("[startKioskTimer] kiosk goal already submitted or undergoing execution, not starting timer");
        return;
      } else if (!agentsWithQuestions.isEmpty()) {
        log.warn("[startKioskTimer] agent has a pending question asked, not starting timer");
        return;
      }
      log.debug("[startKioskTimer] resetting kiosk timer");
      cancelKioskTimer();

      kioskTimerTask = new TimerTask() {
        @Override
        public void run() {
          synchronized (kioskGoalAccessLock) {
            try {
              log.info("[kioskTask] submitting kiosk goal");
              kioskGoalGid = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class, ExecutionType.class, PriorityTier.class)).call(Long.class, kioskGoalPredicate, ExecutionType.ACT, PriorityTier.KIOSK);
            } catch (TRADEException e) {
              log.error("[KioskTimerTask] Issue submitting kiosk goal", e);
            }
          }
        }
      };

      kioskTimer = new Timer(true);
      kioskTimer.schedule(kioskTimerTask, kioskTimeout * 1000);
    }
  }

  // NOTE: Meant to potentially be called by external components. I.e. if the Ava gets a tablet and the timer should be
  //  reset on screen touches, or using camera to detect engagement, etc.
  @TRADEService
  private void resetKioskTimer() {
    log.debug("[resetKioskTimer] in method");
    synchronized (kioskGoalAccessLock) {
      if (!kioskModeEnabled) {
        return;
      } else if (kioskGoalGid != -1) {
        log.debug("[resetKioskTimer] currently in kiosk goal execution and timer was signaled");
        return;
      } else if (!currentGids.isEmpty()) {
        log.debug("[resetKioskTimer] other goals exist and timer was signaled");
        return;
      } else if (!agentsWithQuestions.isEmpty()) {
        log.debug("[resetKioskTimer] pending questions exist and timer was signaled");
        return;
      }
      startKioskTimer();
    }
  }

  @TRADEService
  @Action
  public void setKioskModeEnabled(boolean enabled) {
    log.debug("[setKioskModeEnabled] in method");
    synchronized (kioskGoalAccessLock) {
      if (kioskGoalPredicate == null) {
        log.error("[setKioskModeEnabled] cannot enable kiosk mode without having set a kiosk goal first");
        return;
      }
      kioskModeEnabled = enabled;
      if (enabled) {
        resetKioskTimer();
      } else {
        // Do we want to cancel kiosk goal as well?
        cancelKioskTimer();
      }
    }
  }

  @TRADEService
  @Action
  public boolean getKioskModeEnabled() {
    return kioskModeEnabled;
  }

  @TRADEService
  @Action
  public void setKioskTimeout(long kioskTimeout) {
    log.debug("[setKioskTimeout] in method with timeout {}", kioskTimeout);
    synchronized (kioskGoalAccessLock) {
      this.kioskTimeout = kioskTimeout;
      resetKioskTimer();
    }
  }

  @TRADEService
  @Action
  public boolean setKioskTimeout(Symbol timeSymbol, Symbol timeUnitSymbol) {
    log.debug("[setKioskTimeout] in method with timeout {} {}", timeSymbol, timeUnitSymbol);

    long seconds;
    try {
      long time;
      String timeUnitSymbolString = timeUnitSymbol.getName().toUpperCase();
      float timeFloat = Float.parseFloat(timeSymbol.getName());
      //TODO: address general issue of getting decimal time values
      if (timeFloat == 0.5 && timeUnitSymbolString.equals("HOURS")) {
        time = 30;
        timeUnitSymbolString = "MINUTES";
      } else {
        time = Long.parseLong(timeSymbol.getName());
      }
      TimeUnit timeUnit = TimeUnit.valueOf(timeUnitSymbolString);
      seconds = TimeUnit.SECONDS.convert(time,timeUnit);
    } catch (Exception e) {
      log.error("[setKioskTimeout] could not parse arguments", e);
      return false;
    }

    synchronized (kioskGoalAccessLock) {
      this.kioskTimeout = seconds;
      resetKioskTimer();
    }
    return true;
  }

  @TRADEService
  @Action
  public void setKioskGoal(Predicate goal) {
    log.debug("[setKioskGoal] in method with goal {}", goal);
    synchronized (kioskGoalAccessLock) {
      this.kioskGoalPredicate = goal;
      resetKioskTimer();
    }
  }

  @TRADEService
  @Action
  public long getKioskTimeout() {
    return kioskTimeout;
  }

  @TRADEService
  @Action
  public Predicate getKioskGoalPredicate() {
    return kioskGoalPredicate;
  }

  @TRADEService
  @Action
  public Predicate getWrappedKioskGoalPredicate() {
    return Factory.createPredicate("kioskGoal",kioskGoalPredicate,Factory.createSymbol(String.valueOf(kioskTimeout)));
  }

  @TRADEService
  public void goalStatusChangedCallback(GoalInfo goalInfo) {
    log.debug("[goalStatusChangedCallback] in method {}, {}, {}", goalInfo.gid, goalInfo.unboundGoal, goalInfo.status);
    synchronized (kioskGoalAccessLock) {
      //Should SKIPPENDING goals also trigger a timer reset? I think in general yes, but in conjunction with dialogue
      // listener, maybe not?
      if (goalInfo.priorityTier == PriorityTier.SKIPPENDING) {
        return;
      }
      GoalStatus status = goalInfo.status;
      long gid = goalInfo.gid;
      //Goal complete - check to start kiosk timer
      if (status.isTerminated()) {
        if (gid == kioskGoalGid) {
          log.debug("[goalStatusChangedCallback] kiosk goal terminated, resetting timer");
          kioskGoalGid = -1;
          resetKioskTimer();
        } else {
          currentGids.remove(gid);
          log.debug("[goalStatusChangedCallback] goal terminated, current gids: {}", currentGids);
          if (currentGids.isEmpty()) {
            resetKioskTimer();
          }
        }
      }
      //else {
      //New goal, cancel timer if currently counting down
      else if (status == GoalStatus.PENDING || status == GoalStatus.ACTIVE && gid != kioskGoalGid) {
        currentGids.add(gid);
        log.debug("[goalStatusChangedCallback] goal submitted, current gids: {}", currentGids);
        cancelKioskTimer();
      }
    }
  }

  @TRADEService
  public void dialogueHistoryCallback(Utterance u, boolean newUtterance) {
    log.trace("[dialogueHistoryCallback] in method");
    //TODO: split on listener. In single agentTeam case, any interaction is relevant to the robot
    //Will not cancel existing kiosk goal, only reset submission timer if not executing yet
    resetKioskTimer();

    //Can't do question asking tracking 100% safely from here. Inner actions of other goals could interact with questions,
    //  can't detect questions starting or stopping purely from dialogue (or even dialogue + top-level goals)
  }

  @TRADEService
  public void questionAskingCallback(Symbol agent, boolean active, Question question) {
    log.trace("[questionAskingCallback] in method");

    synchronized (kioskGoalAccessLock) {
      if (active) {
        agentsWithQuestions.add(agent);
      } else {
        boolean hasQuestions = false;
        try {
          hasQuestions = TRADE.getAvailableService(new TRADEServiceConstraints().name("isAgentWaitingForResponse").argTypes(Symbol.class)).call(Boolean.class, agent);
        } catch (TRADEException e) {
          log.error("[questionAskingCallback] unable to query agents QA status");
        }
        if (!hasQuestions) {
          agentsWithQuestions.remove(agent);
        }
      }

      log.debug("[questionAskingCallback] current questions {}", agentsWithQuestions);

      //Interaction kind of weird here. Questions spawned or answered while the kiosk action is active are okay? It's just
      // that a question asked while there is no 'system' goal should block kiosk goal execution so the robot doesn't
      //  potentially leave while a user is interacting with it?
      if (kioskGoalGid == -1) {
        if (agentsWithQuestions.isEmpty()) {
          log.debug("[questionAskingCallback] resetting timer");
          resetKioskTimer();
        } else {
          log.debug("[questionAskingCallback] canceling timer");
          cancelKioskTimer();
        }
      }
    }
  }

  public void setKioskSettingsFile(String filepath) {
    kioskSettingsFile = filepath;
  }

  public void writeKioskSettingsToFile() {
    if (kioskSettingsFile != null) {
      KioskSettings settings = new KioskSettings();
      settings.kioskAction = kioskGoalPredicate;
      settings.kioskTimeout = kioskTimeout;
      settings.enabled = kioskModeEnabled;
      //Same TODO as MapComponent read/write of location references
      File f = new File(kioskSettingsFile);
      try {
        if (!f.exists()) {
          f.getParentFile().mkdirs();
          f.createNewFile();
        }
        ObjectOutputStream os = new ObjectOutputStream(new FileOutputStream(f));
        os.writeObject(settings);
        os.flush();
        os.close();
      } catch (IOException e) {
        log.error("[writeKioskSettingsToFile] error caching current kiosk settings", e);
      }
    }
  }

  private void loadKioskSettings(String filepath) {
    File f = new File(filepath);
    if (!f.exists()) {
      log.warn("[loadKioskSettings] file not found: " + filepath);
      return;
    }
    try {
      ObjectInputStream is = new ObjectInputStream(new FileInputStream(f));
      KioskSettings settings = (KioskSettings) is.readObject();
      setKioskGoal(settings.kioskAction);
      setKioskTimeout(settings.kioskTimeout);
      setKioskModeEnabled(settings.enabled);
      is.close();
    } catch (IOException | ClassNotFoundException e) {
      log.error("[loadKioskSettings] error loading cached kiosk settings on startup", e);
    }
  }

  @Override
  public void shutdownComponent() {
    writeKioskSettingsToFile();
    kioskModeEnabled = false;
    cancelKioskTimer();
  }
}
