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

import java.util.HashSet;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;

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
  private Predicate kioskGoalPredicate;
  private long kioskTimeout; //seconds
  private TimerTask kioskTimerTask;
  private Timer kioskTimer;
  private long kioskGoalGid;
  private Set<Long> currentGids;
  private Set<Symbol> agentsWithQuestions;

  public KioskGoalManager() {
    super();
  };

  @Override
  protected void init() {
    kioskModeEnabled = false;
    kioskGoalPredicate = null;
    kioskTimeout = 60;
    kioskTimer = null;
    kioskTimerTask = null;
    kioskGoalGid = -1;
    currentGids = new HashSet<>();
    agentsWithQuestions = new HashSet<>();
  }

  //TODO: Change so this isn't required to be instantiated after dialogue and GM
  @Override
  protected void postRegistrationInit() {
    //getAgentList()
    registerDialogueListener();
    registerGoalStatusListener();
    registerQuestionAskingListener();

    mockInit();
    if (kioskModeEnabled) {
      resetKioskTimer();
    }
  }

  //TODO: add kiosk annotation to display in App. Add setting to App
  //TODO: CLI args
  private void mockInit() {
    //kioskModeEnabled = true;
    //TODO: better way to make goHomeAndCharge block indefinitely, but also have sane argument for display
    kioskGoalPredicate = Factory.createPredicate("goHomeAndCharge","ava");
    kioskTimeout = 60;
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
    TRADEServiceInfo tsi = getMyService("questionAskingCallback", Symbol.class, Boolean.class);
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
      //TODO: does this need to worry about the kiosk task starting concurrently?
      if (kioskTimerTask != null && kioskTimer != null) {
        log.debug("[cancelKioskTimer] canceling timer and task");
        //TODO: do we need to cancel the timer or the task, or both? what's the difference?
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
          //Do we care about joining on goal here?
          //joinOnGoalDialogue(gid);
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
        //TODO: cancel kiosk goal as well?
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
  public long getKioskTimeout() {
    return kioskTimeout;
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
  public void questionAskingCallback(Symbol agent, boolean active) {
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
}
