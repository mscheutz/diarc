/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.temiv3;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionListener;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.*;
import edu.tufts.hrilab.action.execution.control.ExitContext;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.*;
import java.util.concurrent.atomic.AtomicLong;
import java.util.stream.Collectors;

//TODO: Shouldn't need copies of this if we have multiple GUIs using the same information, change from abstract class
// to concrete class with potentially multiple external listeners for updates
//This class is a start towards disentangling the tracking/storing of general UI relevant information and deciding how
//  that information is actually handled. The immediate motivation for doing this was to get equivalent firebase
//  functionality working on the temi as our desktop setup without needing to have a separate implementation of the entire
//  FirebaseConnectionComponent. It is also a step in the right direction anyway. WIP
public abstract class UIComponent extends DiarcComponent implements ActionListener {
  private boolean connectToAction;
  private boolean connectToBelief;
  private boolean connectedToBelief;
  private boolean connectedToAction;
  private boolean connectToDialogue;
  private boolean connectedToDialogue;
  private boolean registeredAIListener;

  /**
   * Default db file directory to make command line args cleaner.
   */
  private String dbFileDir = "config/edu/tufts/hrilab/action/asl";
  private List<String> actionsToWrite;
  private List<String> actionFilesToWrite;

  private Set<String> relevantActions;
  private HashMap<Context, Context> trackedContexts;

  private final Object goalRepLock = new Object();
  private List<String> pendingGoalStrings;
  private List<Long> pendingGoalIds;
  private List<GoalStatus> pendingGoalStatuses;

  private Map<String, List<Map<Variable, Symbol>>> beliefState;

  private TreeSet<String> dictKeys;

  //TODO: get rid of this?
  private static AtomicLong orderCounter = new AtomicLong();

  public UIComponent() {
    super();

    connectToBelief = false;
    connectedToBelief = false;
    connectedToAction = false;
    connectToDialogue = false;
    connectedToDialogue = false;
    registeredAIListener = false;

    connectToAction = false;
    actionsToWrite = new ArrayList<>();
    actionFilesToWrite = new ArrayList<>();

    relevantActions = new HashSet<>();
    trackedContexts = new HashMap<>();

    beliefState = new HashMap<>();

    dictKeys = null;

    pendingGoalStrings = new ArrayList<>();
    pendingGoalIds = new ArrayList<>();
    pendingGoalStatuses = new ArrayList<>();

    shouldRunExecutionLoop = true;
  }

  @Override
  protected void init() {
    //Demo specific - make configurable
    populateRelevantActions();

    //Initialize Dictionary keys in firebase. Used to allow the user to select base morphemes for homophone definitions
    //TODO: Have some sort of mechanism to attach a listener to the Dictionary to update this list on new definitions
    //  Looks like we previously had an unsafe call from the parser in injectDictionaryEntry which we don't want
    try {
      log.debug("[FirebaseConnectionComponent] initializing new dictionary key collection on firebase");
      dictKeys = new TreeSet<>(TRADE.getAvailableService(new TRADEServiceConstraints().name("getDictionaryEntries")).call(Set.class));
      onDictionaryEntriesUpdated(dictKeys);
    } catch (TRADEException e) {
      log.error("[FirebaseConnectionComponent] error calling getDictionaryEntries", e);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("action").desc("register for action status notifications to push execution status and generated asl to firebase").build());
    options.add(Option.builder("dialogue").desc("make calls to dialogue services").build());
    options.add(Option.builder("dbfileDir").hasArg().argName("dir").desc("set default db file directory so filenames (without path) can be passed with -dbfile").build());
    options.add(Option.builder("dbfile").hasArgs().argName("file").desc("db files for which to push ASL to firebase").build());
    options.add(Option.builder("dbaction").hasArgs().argName("name").desc("action names for which to push ASL to firebase").build());
    options.add(Option.builder("belief").desc("register for belief notifications and write result to firebase").build());

    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("dialogue")) {
      connectToDialogue = true;
    }
    if (cmdLine.hasOption("dbfileDir")) {
      dbFileDir = cmdLine.getOptionValue("dbfileDir");
    }
    if (cmdLine.hasOption("dbfile")) {
      List<String> actionFiles = Arrays.asList(cmdLine.getOptionValues("dbfile"));
      for (String actionFile : actionFiles) {
        actionFilesToWrite.add(Resources.createFilepath(dbFileDir, actionFile));
      }
    }
    if (cmdLine.hasOption("dbaction")) {
      actionsToWrite.addAll(Arrays.asList(cmdLine.getOptionValues("dbaction")));
    }
    if (cmdLine.hasOption("belief")) {
      connectToBelief = true;
    }
    if (cmdLine.hasOption("action")) {
      connectToAction = true;
    }
  }

  @Override
  protected void executionLoop() {
    if ((connectToAction && !connectedToAction) || (!connectedToBelief && connectToBelief) || (connectToDialogue && !connectedToDialogue) || !registeredAIListener) {

      //Get New Action Notifications (on startup, learned, plans)
      if (connectToAction && !connectedToAction) {
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("registerNotification")).call(void.class, getMyService("onNewActionsCallback", List.class));
          connectedToAction = true;
        } catch (TRADEException e) {
          log.error("[FirebaseConnectionComponent] Error registering for action notifications: ", e);
        }
      }

      if (!connectedToBelief && connectToBelief) {
          try {
            //TODO: foodOrdering specific, make this configurable
            TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForNotification")).call(void.class, Factory.createPredicate("meal", "X", "Y"), getMyService("beliefNotificationCallback", Term.class, List.class));
            TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForNotification")).call(void.class, Factory.createPredicate("mealReady", "X", "Y"), getMyService("beliefNotificationCallback", Term.class, List.class));
            connectedToBelief = true;
          } catch (TRADEException e) {
            log.error("[FirebaseConnectionComponent] Error registering for AI notifications: ", e);
          } catch (NullPointerException ignored) {
          }
      }
      //Get Dialogue History Notifications
      if (connectToDialogue && !connectedToDialogue) {
          try {
            TRADEServiceInfo updateFirebaseDialogueHistoryService = getMyService("updateFirebaseDialogueHistory", Utterance.class);
            TRADE.getAvailableService(new TRADEServiceConstraints().name("registerForDialogueHistoryNotifications")).call(void.class, updateFirebaseDialogueHistoryService);
            connectedToDialogue = true;
          } catch (TRADEException e) {
            log.error("exception registering for dialogue notifications", e);
          }
      }

      //Get Notifications on AI start/end and cycle progress
      if (!registeredAIListener) {
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("registerAIListener")).call(void.class, this);
          registeredAIListener = true;
        } catch (TRADEException e) {
          log.error("[FirebaseConnectionComponent] Error registering for AI notifications: ", e);
        }
      }
    }
  }

  //Abstract methods - handle information potentially relevant to UI updates
  @TRADEService
  public abstract void onAgentActionUpdated(String actor, String goal, String status);

  //QueueExecutionManager only - change impl to be more generic, use pending/active goals collections from base EM
  //  rather than QueueEM specific info/methods
  //@TRADEService
  //public abstract void onSystemGoalsUpdated(Map<String,Object> goalInfo);
  @TRADEService
  public abstract void onActiveGoalUpdated(String goalString, GoalStatus status, long gid);

  @TRADEService
  public abstract void onPendingGoalsUpdated(List<String> goalStrings, List<GoalStatus> statuses, List<Long> gids);

  @TRADEService
  public abstract void onActionGenerated(ActionDBEntry action, boolean onStartup);

  @TRADEService
  public abstract void onBeliefNotificationUpdate(String queryTermString, Map<String, Object> bindingsStrings);

  @TRADEService
  public abstract void onDictionaryEntriesUpdated(Set<String> keys);

  //Called from EM on updates, change how this works
  @TRADEService
  public void notifyPendingGoalUpdated(Goal g, int index, boolean added) {
    synchronized (goalRepLock) {
      if (added) {
        pendingGoalStrings.add(index, convertGoalToReadableString(g));
        pendingGoalIds.add(index, g.getId());
        pendingGoalStatuses.add(index, g.getStatus());
      } else {
        pendingGoalStrings.remove(index);
        pendingGoalIds.remove(index);
        pendingGoalStatuses.remove(index);
      }
    }
    onPendingGoalsUpdated(pendingGoalStrings, pendingGoalStatuses, pendingGoalIds);
  }

  @TRADEService
  public void notifyActiveGoalUpdated(Goal goal, GoalStatus status) {
    onActiveGoalUpdated(convertGoalToReadableString(goal), status, goal.getId());
  }

  private void onAgentActionUpdated(String actor, Predicate goal, String status) {
    String goalPred = goal.toString();
    goalPred = modifyPredStringToReadable(goalPred);
    if (status.equals("SUCCEEDED")) {
      status = "SUCCESS";
    }
    onAgentActionUpdated(actor, goalPred, status);
  }

  //Listener registered with the goal manager to be notified of new actions. This triggers both immediately on startup
  //  (with all actions loaded through the current config) and on any newly learned action or plan
  @TRADEService
  public void onNewActionsCallback(List<ActionDBEntry> actions) {
    log.debug("[onNewActionCallback] with actions " + actions);
    for (ActionDBEntry action : actions) {
      if (action.isScript()) {
        //Is the only case where a script won't have an associated dbfile when it is a taught action or a plan?
        //Do we want to always display all plans and taught actions, or need to specify which ones to display or omit?
        if (action.getDBFile() == null) {
          onActionGenerated(action, false);
          if (!action.getName().equals("planned")) {
            addRelevantDisplayAction(action.getName());
          }
        } else if (actionsToWrite.contains(action.getName())) {
          onActionGenerated(action, true);
        } else {
          //TODO: if filepath corresponds to directory where saved actions from webapp will be saved, then write to firebase
          String[] filepath = action.getDBFile().split("/");
          //second part of conditionial is the only way as of now to tell whether this is an action modified and resubmitted through the webapp
          if (actionFilesToWrite.contains(action.getDBFile()) || filepath[filepath.length - 1].startsWith("learnedActions")) {
            onActionGenerated(action, true);
          }
        }
      }
    }
  }

  //TODO: revisit this - implementation is rushed and right now only specific to foodOrdering
  //Listener registered with belief to receive notifications on updates to the supplied bindings. The relevant state is
  //  tracked in this component and written to firebase on update for display. Currently used in foodOrdering demo for
  //  tracking meal options and completed meals.
  @TRADEService
  public void beliefNotificationCallback(Term query, List<Map<Variable, Symbol>> bindings) {
    log.info("[beliefNotificationCallback] " + query + " : " + bindings);
    String queryString;
    List<Map<Variable, Symbol>> updatedBindings = null;
    //Internally tracking current notification state as map with keys as registered belief queries and values as complete sets of bindings
    //If retraction, update current state by removing all bindings for query which are no longer true.
    //Otherwise, update or initialize existing entry of corresponding query
    if (query.isNegated()) {
      queryString = query.toUnnegatedForm().toString();
      List<Map<Variable, Symbol>> unnegatedBindings = beliefState.get(queryString);
      if (unnegatedBindings != null) {
        unnegatedBindings.removeAll(bindings);
        if (!unnegatedBindings.isEmpty()) {
          updatedBindings = beliefState.get(queryString);
        }
      } else {
        log.warn("[beliefNotificationCallback] got retraction notification for query not in belief state");
        return;
      }
    } else {
      queryString = query.toString();
      if (beliefState.containsKey(queryString)) {
        beliefState.get(queryString).addAll(bindings);
      } else {
        beliefState.put(queryString, bindings);
      }
      updatedBindings = beliefState.get(queryString);
    }

    Map<String, Object> response = new HashMap<>();
    List<String> bindingsStrings = new ArrayList<>();
    //Consider tradeoffs to doing it this way or doing conversion before updating beliefState and holding map<String,List<String>>
    if (updatedBindings != null) {
      bindingsStrings = updatedBindings.stream().map(this::convertBindingsToString).collect(Collectors.toList());
    }
    response.put("bindings", bindingsStrings);
    onBeliefNotificationUpdate(queryString, response);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Action Listener/Status Tracking Start ////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Relevant top level goals are currently tracked in the QueueExecutionManager
  @Override
  public void actionStarted(ActionInterpreter ai) {
  }

  @Override
  public void actionComplete(ActionInterpreter ai) {
    //TODO: Can this occur arbitrarily deep into nested script execution or would it be covered in stepComplete for lower level actions?
    //If top level script gets canceled the child stored in trackedContexts never gets executed and therefore updated/removed otherwise
    Goal goal = ai.getGoal();
    if (goal != null && GoalStatus.CANCELED.equals(goal.getStatus())) {
      ChildContexts children = ai.getRoot().getChildContexts();
      if (!children.isEmpty()) {
        Context step = children.get(0);
        if (step != null && shouldProcessAction(step.getSignatureInPredicateForm())) {
          for (Map.Entry<Context, Context> entry : trackedContexts.entrySet()) {
            if (entry.getValue().equals(step)) {
              Context parent = trackedContexts.remove(entry.getKey());
              if (parent != null) {
                Symbol parentActor = (Symbol) parent.getArgumentValue("?actor");
                onAgentActionUpdated(parentActor.getName(), parent.getSignatureInPredicateForm(), parent.getStatus().toString());
              }
              return;
            }
          }
        }
      }
    }
  }

  @Override
  public void stepStarted(Context step) {
    //Prevent calls to PA from within this file from affecting tracking state
    if (step.getExecType().isSimulation()) {
      return;
    }

    if (step instanceof ActionContext) {
      //Disregard actions which we do not care about displaying
      if (!shouldProcessAction(step.getSignatureInPredicateForm())) {
        return;
      }

      //This does not handle a single agent having multiple concurrent goals, overwrites current entry if it exists
      Symbol actor = (Symbol) step.getArgumentValue("?actor");
      onAgentActionUpdated(actor.getName(), step.getSignatureInPredicateForm(), "PROGRESS");
    }
  }

  @Override
  public void stepComplete(Context step) {
    log.trace("[stepComplete] " + step.getCommand());

    //Prevent calls to PA from within this file from affecting tracking state
    if (step.getExecType().isSimulation()) {
      return;
    }

    //TODO: need to revisit this and test for all cases
    //TODO: this works the multiRobotCaddy demo, but I don't know if in general a script exiting
    //  necessarily means that whatever parent we match against has failed/stopped execution
    //Special case if we exit a script
    if (step instanceof ExitContext || step instanceof FailureContext) {
      Context parentContext = step.getParentContext();
      while (!(parentContext instanceof RootContext)) {
        Symbol actor = (Symbol) parentContext.getArgumentValue("?actor");
        if (parentContext instanceof ActionContext && shouldProcessAction(parentContext.getSignatureInPredicateForm())) {
          onAgentActionUpdated(actor.getName(), parentContext.getSignatureInPredicateForm(), parentContext.getStatus().toString());
        }
        parentContext = parentContext.getParentContext();
      }
    }


    //TODO: handle goal states directly as well?
    //General case - Action Context has terminated
    if (step instanceof ActionContext) {
      //Only act on actions which we do care about displaying
      //Assuming 'planned' is in this list
      if (shouldProcessAction(step.getSignatureInPredicateForm())) {
        //Action is a script, wait for completion of last child to update terminal status
        if (((ActionContext) step).isScript() && step.getChildContexts().size() > 0) {
          //Store last child context and wait for its termination to update status of the parent. See notes below
          ChildContexts children = step.getChildContexts();
          if (trackedContexts.containsKey(children.size() - 1)) {
            //EW: let me know if this occurs and when. Believe this is possible but don't have time to implement and test right now
            log.warn("overwriting tracked context");
          }
          trackedContexts.put(children.get(children.size() - 1), step);
        }
        //Action is a primitive, terminated now - Update display
        else {
          Symbol actor = (Symbol) step.getArgumentValue("?actor");
          onAgentActionUpdated(actor.getName(), step.getSignatureInPredicateForm(), step.getStatus().toString());
        }
      }
    }

    //Not an action we directly want to write/store the status of, but we still need to check if an ancestor script
    // we are tracking is pending termination of this step. The step relating directly to a script terminates
    // immediately after setupSteps is called in ActionContext. This populates the children contexts and returns, but
    // the script itself only really completes when the last child context has completed execution. This execution flow
    // is potentially recursive as any script's last step can be another script.
    Context parent = trackedContexts.remove(step);
    if (parent != null) {
      //If last child of a script is a script, we need to recurse while maintaining the original parent as the action
      //  whose status to update
      if (((step instanceof ActionContext && ((ActionContext) step).isScript()) || step instanceof GoalContext) && step.getChildContexts().size() > 0) {
        ChildContexts children = step.getChildContexts();
        trackedContexts.put(children.get(children.size() - 1), parent);
      }
      //Otherwise, this step fully terminated and we can update the status of the dependent script waiting on this
      else {
        Symbol parentActor = (Symbol) parent.getArgumentValue("?actor");
        //Is it safe to assume if we haven't reached an ExitContext in the script, that it succeeded?
        onAgentActionUpdated(parentActor.getName(), parent.getSignatureInPredicateForm(), "SUCCESS");
      }
      if (trackedContexts.remove(step) != null) {
        //EW: let me know if this occurs and when. Believe this is possible but don't have time to implement and test right now
        log.warn("second entry for context exists");
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Action Listener/Status Tracking End /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //TODO: configure this according to demo somewhere more appropriate
  //Initialize list of 'relevant' action for the current DIARC instance. This list is used by action listeners in this
  //  component to determine whether actions' corresponding status updates should be processed and written to relevant
  //  documents on firebase for display or ignored
  private void populateRelevantActions() {
    //assembly
    relevantActions.add("recordPose");
    relevantActions.add("init");
    relevantActions.add("addDetectionType");
    relevantActions.add("setupPoses");
    relevantActions.add("openGripper");
    relevantActions.add("goToPose");
    relevantActions.add("defineScrewType");
    relevantActions.add("configureScrewdriverProgram");
    relevantActions.add("defineItem");
    relevantActions.add("startLearningAssembleScript");
    relevantActions.add("endLearningAssembleScript");
    relevantActions.add("assemblenv30fau");
    relevantActions.add("goToCameraPose");
    relevantActions.add("perceiveEntity");
    relevantActions.add("getOn");
    relevantActions.add("moveAndOrientToCognexTarget");
    relevantActions.add("putdown");
    relevantActions.add("observeDescriptor");
    relevantActions.add("screwIn");
    relevantActions.add("mountScrew");
    relevantActions.add("mountSingleScrew");
    relevantActions.add("moveToCognexTarget");
    relevantActions.add("alignWith");
    relevantActions.add("runScrewdriverJob");
    relevantActions.add("grab");
    relevantActions.add("planned");
    relevantActions.add("closeGripper");
    relevantActions.add("moveConveyorForward");
    relevantActions.add("modifyAssemble");
    relevantActions.add("modifyAction");
    relevantActions.add("assemblenf32sv");
    relevantActions.add("moveToObjectHeight");
    relevantActions.add("moveToCameraHeight");
    relevantActions.add("rotateToEE");

    //foodOrdering
    relevantActions.add("init");
    relevantActions.add("orderMeal");
    relevantActions.add("prepareMeal");
    relevantActions.add("defineMeal");
    relevantActions.add("defineItem");
    relevantActions.add("defineTypeOfOrderItem");
    relevantActions.add("defineIngredient");
    relevantActions.add("updateActionLearning");
    relevantActions.add("askQuestion");
    relevantActions.add("waitForResponse");
    relevantActions.add("goToLocation");
    relevantActions.add("perceiveitem");
    relevantActions.add("pickupItem");
    relevantActions.add("putDownItem");
    relevantActions.add("putItemOn");
    relevantActions.add("fry");
    relevantActions.add("splitBun");
    relevantActions.add("cook");
    relevantActions.add("saveCurrentLocation");
    relevantActions.add("saveCurrentPose");
    relevantActions.add("saute");
    relevantActions.add("freeze"); //rename in display
    relevantActions.add("defineIngredientHelper");
    relevantActions.add("defineItemByAnalogy");
    relevantActions.add("followMeBlocking");
    relevantActions.add("supersedeSystemGoal");
    relevantActions.add("modifyPrepare");

    //temi
    relevantActions.add("relocalize");
    relevantActions.add("followMe");
    relevantActions.add("followMeBlocking");
    relevantActions.add("goToLocation");
    relevantActions.add("saveLocation");
    relevantActions.add("deleteLocation");
    relevantActions.add("display");
    relevantActions.add("sayText");
    relevantActions.add("playVideo");
    relevantActions.add("escort");
    relevantActions.add("fetch");
    relevantActions.add("greet");
    relevantActions.add("tour");
    relevantActions.add("clearScreen");
    relevantActions.add("displayFace");
    relevantActions.add("chargeTemi");
    relevantActions.add("goToThenSay");
    relevantActions.add("goToThenDisplay");
    relevantActions.add("goToThenPlay");
    relevantActions.add("setVolume");
    relevantActions.add("displayQRCode");
    relevantActions.add("relocalize");
  }

  //Demo specific
  private boolean shouldProcessAction(Predicate action) {
    String name = action.getName();

    return relevantActions.contains(name);
  }

  @TRADEService
  public void addRelevantDisplayAction(String name) {
    relevantActions.add(name);
  }

  private String convertGoalToReadableString(Goal g) {
    if (g == null) {
      return "None";
    } else {
      return modifyPredStringToReadable(g.getPredicate().toString());
    }
  }

  private String modifyPredStringToReadable(String predString) {
    String[] splitRes = predString.split("[(),](?=[^\"][^\"(),]*[^\"][(),]|\".*\"(?![^,()]))");

    //assembly specific
    switch (splitRes[0]) {
      case "startLearningAssembleScript":
      case "endLearningAssembleScript":
      case "perceiveEntityFromSymbol":
      case "assemblenv30fau":
        return splitRes[0];
      case "askQuestion":
        return splitRes[0] + " " + splitRes[3];
      case "planned":
        return "execute plan";
    }

    //Trim end parens from last arg - sloppy, do this better
    int resLen = splitRes.length;
    while (splitRes[resLen - 1].endsWith(")")) {
      splitRes[resLen - 1] = splitRes[resLen - 1].substring(0, splitRes[resLen - 1].length() - 1);
    }

    //Temi specific, move this
    String funcName = splitRes[0];
    switch (funcName) {
      case "followMe":
      case "followMeBlocking":
        return "Follow";
      case "goToLocation":
        return String.format("Go to location '%s'", convertArgToReadable(splitRes[2]));
      case "saveLocation":
        return String.format("Save location '%s'", splitRes[2]);
      case "deleteLocation":
        return String.format("Delete location '%s'", convertArgToReadable(splitRes[2]));
      case "display":
        return String.format("Display message '%s'", splitRes[2]);
      case "sayText":
        return String.format("Say message '%s'", splitRes[2]);
      case "playVideo":
        return String.format("Play video '%s'", splitRes[2]);
      case "escort":
        if (splitRes.length == 6) {
          return String.format("Escort %s from %s to %s", splitRes[3], convertArgToReadable(splitRes[4]), convertArgToReadable(splitRes[2]));
        } else {
          return String.format("Escort %s from current location to %s", splitRes[3], convertArgToReadable(splitRes[2]));
        }
      case "fetch":
        if (splitRes.length == 5) {
          return String.format("Fetch %s from %s", splitRes[2], convertArgToReadable(splitRes[3]));
        } else {
          return String.format("Fetch %s", splitRes[2]);
        }
      case "greet":
        return "Greet in " + splitRes[2];
      case "tour":
        return "Tour";
      case "clearScreen":
        return "Clearing Screen";
      case "displayFace":
        return "Display Face";
      case "chargeTemi":
        return "Charging";
      case "goToThenSay":
        return String.format("Go to %s then say message", convertArgToReadable(splitRes[2]));
      case "goToThenDisplay":
        return String.format("Go to %s then display message", convertArgToReadable(splitRes[2]));
      case "goToThenPlay":
        return String.format("Go to %s then play video %s", convertArgToReadable(splitRes[2]), splitRes[3]);
      case "setVolume":
        return String.format("Set volume to %s", splitRes[2]);
      case "displayQRCode":
        return "Display Website QR";
      case "relocalize":
        return "Reposition";
      case "checkInKiosk":
        return String.format("Check in patients at location %s", convertArgToReadable(splitRes[2]));
    }

    //TODO: currently don't handle nested predicates correctly
    List<String> splitResArray = new ArrayList<>(List.of(splitRes));
    splitResArray.removeIf(e -> e.equals(""));

    //remove actor
    if (resLen >= 2) {
      splitResArray.remove(1);
    }

    //TODO: allow for other components to define how to convert a certain predicate in a custom way?
    StringBuilder out = new StringBuilder(splitResArray.remove(0));
    for (String arg : splitResArray) {
      out.append(" ").append(convertArgToReadable(arg));
    }
    return out.toString();
  }

  //dereference arg if neceesary
  public String convertArgToReadable(String arg) {
    String[] splitRes = arg.split(":");
    if (splitRes.length > 1) {
      Symbol argSymbol = Factory.createSymbol(arg);
      if ("agent".equals(splitRes[1]) || "property".equals(splitRes[1])) {
        return argSymbol.getName();
      }
      try {
        List<Term> properties = TRADE.getAvailableService(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class)).call(List.class, argSymbol);
        String ret = properties.stream().map(Term::getName).collect(Collectors.joining(" "));
        return ret;
      } catch (TRADEException ignored) {
      }
      return argSymbol.getName();
    }
    return arg;
  }

  //Helper method for beliefNotificationCallback
  //Consider if it's worth doing this here/in java or just process externally in the gui
  private String convertBindingsToString(Map<Variable, Symbol> bindings) {
    Symbol yBinding = bindings.get(new Variable("Y"));
    if (yBinding != null && yBinding.getName().equals("orderItems")) {
      Predicate orderItems = (Predicate) yBinding;
      StringBuilder ret = new StringBuilder(bindings.get(new Variable("X")).toString());
      for (Symbol orderItem : orderItems.getArgs()) {
        ret.append(",").append(orderItem.toString());
      }
      return ret.toString();
    } else if (yBinding != null && yBinding.getName().equals("items")) {
      long orderId = orderCounter.incrementAndGet();
      Predicate mealComponents = (Predicate) yBinding;
      try {
        //TODO: have additional items( around each items(components)
        StringBuilder ret = new StringBuilder();
        ret.append("Order ").append(orderId).append(": ");
        int i = 0;
        for (Symbol component : mealComponents.getArgs()) {
          if (i != 0) {
            ret.append(", ");
          }
          i += 1;
          List<Term> properties = TRADE.getAvailableService(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class)).call(List.class, Factory.createSymbol(component.getName(), component.getName().split("_")[0]));
          //TODO; revisit this
          String itemString = properties.stream().map(Term::toString).collect(Collectors.joining(" "));
          ret.append(itemString.split("\\(")[0]);
        }
        return ret.toString();
      } catch (TRADEException e) {
        return bindings.toString();
      }
    } else {
      return bindings.toString();
    }
  }
}
