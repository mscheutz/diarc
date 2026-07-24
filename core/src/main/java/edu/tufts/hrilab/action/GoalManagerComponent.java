/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import com.google.gson.Gson;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;
import com.google.gson.reflect.TypeToken;
import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.description.ContextDescription;
import edu.tufts.hrilab.action.execution.*;
import edu.tufts.hrilab.action.goal.*;
import edu.tufts.hrilab.action.gui.GoalManagerAdapter;
import edu.tufts.hrilab.action.gui.GoalViewerAdapter;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.lock.ActionResourceLockLinear;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.action.manager.PriorityInfo;
import edu.tufts.hrilab.action.notification.GoalManagerNotifier;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.action.planner.pddl.PddlParser;
import edu.tufts.hrilab.action.selector.ActionSelector;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.GuiProvider;
import edu.tufts.hrilab.util.Util;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.*;
import java.util.stream.Collectors;

import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;


/**
 * A class responsible for managing and executing goals in a system. The `GoalManagerComponent` interfaces with
 * goal priorities, constraints, database files, and PDDL files to manage goal submission, execution, and status
 * tracking. The class supports functionality for initiating goals, parsing input files, assigning priorities,
 * and handling goal lifecycle operations such as submission, suspension, resumption, and cancellation.
 *
 * Fields:
 * - em: Execution manager type.
 * - executionManagerType: Type for execution management of goals.
 * - priorityFile: File containing goal priority information.
 * - initGoals: A list of initial goals to be executed.
 * - initialConstraints: List of constraints to be applied during initialization.
 * - dbFiles: List of specified database files for goals.
 * - pddlFireDir: Directory for PDDL file execution.
 * - pddlFiles: List of PDDL domain and problem files.
 * - performanceFiles: Files containing performance logs and metrics.
 * - dbFileDir: Directory for database file storage.
 * - performanceFileDir: Directory for performance file storage.
 * - goalPriorities: Priority assignments for goals.
 * - displayActionLearningGui: Flag for displaying the action learning GUI.
 * - beliefArgs: Belief arguments for goal parsing.
 * - notifier: Component for goal-related notifications or updates.
 * - useMemoryManager: Flag enabling/disabling usage of the memory manager.
 * - historyLength: Configurable length for stored goal history.
 * - contextConsultant: Consultant for contextual goal management.
 *
 * Methods:
 * - `GoalManagerComponent`: Constructs a new instance of the goal manager component.
 * - `additionalUsageInfo`: Provides a list of additional usage information for this component.
 * - `parseArgs`: Parses command-line arguments specific to this class.
 * - `init`: Initializes the goal manager component.
 * - `initializeGM`: Prepares the goal manager during construction.
 * - `initializeDB`: Initializes the database for goal management.
 * - `addInitGoal`: Adds a goal from a command-line string to the list of initial goals to be executed.
 * - `submitInitGoals`: Submits the initial goals specified during setup for execution.
 * - `loadPDDLFiles`: Parses PDDL domain and problem files and adds the relevant data (actions, facts, goals) to the system.
 * - `loadGoalPriorities`: Attempts to load goal priority data from a JSON file.
 * - `getPriorityTierForGoal`: Retrieves the default priority tier for a specified goal predicate.
 * - `getPriorityForGoal`: Retrieves the default priority value for a specified goal predicate.
 * - `submitGoal`: Submits a goal to be achieved, with overloaded methods supporting additional parameters like execution type, priority tier, and priority value.
 * - `submitGoalWithMetric`: Submits a goal with a specific metric for planning purposes.
 * - `cancelGoal`: Cancels a goal by its unique identifier.
 * - `cancelAllCurrentGoals`: Cancels all current pending and active goals, excluding the "listen goal."
 * - `cancelAllActiveGoals`: Cancels all currently active goals, excluding the "listen goal."
 * - `cancelAllPendingGoals`: Cancels all goals currently in the pending state.
 * - `cancelPendingGoalByIndex`: Cancels a pending goal identified by its index in descending priority order.
 * - `suspendGoal`: Suspends an active or pending goal by its unique identifier.
 * - `resumeGoal`: Resumes a suspended goal identified by its unique identifier.
 * - `get*/
public class GoalManagerComponent extends DiarcComponent implements GuiProvider {
  /**
   * ExecutionManager instance.
   */
  private ExecutionManager em;
  /**
   * ExecutionManager class type
   */
  private Class<ExecutionManager> executionManagerType = ExecutionManager.class;
  /**
   * File defining priority information values for goals
   */
  private String priorityFile = "default.json";
  /**
   * Scripts or goals to instantiate on start-up. Goal predicates hashed by actor.
   */
  private List<Predicate> initGoals = new ArrayList<>();
  /**
   * Initial constraints on what actions/states are permissible or forbidden.
   */
  private ActionConstraints initialConstraints = new ActionConstraints();
  /**
   * Action script files to load into the database.
   */
  private List<String> dbFiles = new ArrayList<>();
  /**
   * Default pddl file directory to make command line args cleaner.
   */
  private String pddlFireDir = "config/edu/tufts/hrilab/action/pddl";
  /**
   * PDDL domain and problem file to load into the DIARC.
   */
  private Pair<String, String> pddlFiles;
  /**
   * Action Performance Measures files to load into the database
   */
  private List<String> performanceFiles = new ArrayList<>();
  /**
   * Default db file directory to make command line args cleaner.
   */
  private String dbFileDir = "config/edu/tufts/hrilab/action/asl";
  /**
   * Default performance measures directory to make command line args cleaner.
   */
  private String performanceFileDir = "config/edu/tufts/hrilab/action/performancemodels";
  /**
   * A map of goal names to priority information (tier and value) to be used by the execution manager to order goals.
   * These are loaded from json files found in config.action.manager.priority
   */
  private Map<String, PriorityInfo> goalPriorities;
  /**
   * Action learning gui flag.
   */
  private boolean displayActionLearningGui = false;
  /**
   * BeliefComponent args to pass along during instantiation.
   */
  private List<String> beliefArgs = new ArrayList<>();
  /**
   * Notifier for addition/removal of actions.
   */
  private GoalManagerNotifier notifier;

  /**
   * Command line arg to utilize pruning of memory in goal manager
   */
  private boolean useMemoryManager = true;

  /**
   * Command line arg specify how pruning of memory in goal manager
   */
  private long historyLength = 60000;

  /**
   * The context consultant allows a human interactor to refer to previously
   * executed actions through the NL pipline
   */
  private ContextConsultant contextConsultant;

  /**
   * Constructs the GoalManagerComponent.
   */
  public GoalManagerComponent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("asl").longOpt("dbfile").hasArgs().argName("agent1,agentN:file").desc("parse ASL file for (optional) agent(s) agent1,agentN").build());
    options.add(Option.builder("pddl").numberOfArgs(2).argName("domainFile problemFile").desc("parse PDDL domain and problem file").build());
    options.add(Option.builder("dbfileDir").hasArg().argName("dir").desc("set default db file directory so filenames (without path) can be passed with -dbfile").build());
    options.add(Option.builder("performancefile").hasArgs().argName("file").desc("parse performance models file").build());
    options.add(Option.builder("performancefileDir").hasArg().argName("dir").desc("set default action performance directory so filenames (without path) can be passed with -performancefile").build());
    options.add(Option.builder("selector").hasArg().argName("classpath").desc("set action selector type (must be full classpath)").build());
    options.add(Option.builder("learningGui").desc("display learning gui").build());
    options.add(Option.builder("goal").hasArgs().argName("goal-pred").desc("start goal with specified goal predicate. \"goal(actor,state)\" or \"action(actor,args)\"").build());
    options.add(Option.builder("badaction").numberOfArgs(1).desc("add bad action").build());
    options.add(Option.builder("badstate").numberOfArgs(1).desc("add bad state").build());
    options.add(Option.builder("executionManagerType").hasArgs().desc("fully qualified type for execution manager").build());
    options.add(Option.builder("priorityFile").hasArgs().desc("goal priority information file").build());
    options.add(Option.builder("nomemorymanager").desc("Do not utilize the memory manager in the goal manager to prune old goals").build());
    options.add(Option.builder("historylength").hasArg().argName("time").desc("How long to keep past goals").build());

    // local belief component options
    options.add(Option.builder("beliefinitfile").hasArgs().argName("file").desc("load initialization file").build());
    options.add(Option.builder("beliefworking").hasArgs().argName("file").desc("load working file").build());
    options.add(Option.builder("beliefepisodic").hasArgs().argName("file").desc("load episodic file").build());
    options.add(Option.builder("beliefuniversal").hasArgs().argName("file").desc("load universal file").build());
    options.add(Option.builder("beliefprover").hasArg().argName("enum").desc("set prover type").build());
    options.add(Option.builder("beliefdisk").hasArg().argName("databasepath").desc("use existing sql disk located at path").build());
    options.add(Option.builder("beliefg").desc("show belief gui").build());
    options.add(Option.builder("beliefgroups").hasArgs().desc("TRADE groups for belief component").build());

    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("executionManagerType")) {
      try {
        executionManagerType = (Class<ExecutionManager>) Class.forName(cmdLine.getOptionValue("executionManagerType"));
      } catch (ClassNotFoundException | SecurityException | IllegalArgumentException | ClassCastException e) {
        log.error("Exception in setting ExecutionManager type: " + cmdLine.getOptionValue("executionManagerType") +
                ". Using default type instead.", e);
      }
    }
    if (cmdLine.hasOption("priorityFile")) {
      priorityFile = cmdLine.getOptionValue("priorityFile");
    }
    if (cmdLine.hasOption("badaction")) {
      initialConstraints.addForbiddenAction(cmdLine.getOptionValue("badaction"));
    }
    if (cmdLine.hasOption("badstate")) {
      Predicate bad = Factory.createPredicate(cmdLine.getOptionValue("badaction"));
      initialConstraints.addForbiddenState(bad);
    }
    if (cmdLine.hasOption("selector")) {
      ActionSelector.setActionSelectorType(cmdLine.getOptionValue("selector"));
    }
    if (cmdLine.hasOption("goal")) {
      Arrays.asList(cmdLine.getOptionValues("goal")).forEach(goal -> addInitGoal(goal));
    }
    if (cmdLine.hasOption("asl")) {
      dbFiles.addAll(Arrays.asList(cmdLine.getOptionValues("asl")));
    }
    if (cmdLine.hasOption("dbfileDir")) {
      dbFileDir = cmdLine.getOptionValue("dbfileDir");
    }
    if (cmdLine.hasOption("pddl")) {
      String[] files = cmdLine.getOptionValues("pddl");
      pddlFiles = new ImmutablePair<>(files[0], files[1]);
    }
    if (cmdLine.hasOption("performancefile")) {
      performanceFiles.addAll(Arrays.asList(cmdLine.getOptionValues("performancefile")));
    }
    if (cmdLine.hasOption("performancefiledir")) {
      performanceFileDir = cmdLine.getOptionValue("performancefileDir");
    }
    if (cmdLine.hasOption("learningGui")) {
      displayActionLearningGui = true;
    }
    if (cmdLine.hasOption("nomemorymanager")) {
      useMemoryManager = false;
    }
    if (cmdLine.hasOption("historylength")) {
      historyLength = Long.parseLong(cmdLine.getOptionValue("historylength"));
    }

    /////////////////////////////////
    // local belief component options
    /////////////////////////////////

    if (cmdLine.hasOption("beliefinitfile")) {
      beliefArgs.add("-initfile");
      beliefArgs.addAll(Arrays.asList(cmdLine.getOptionValues("beliefinitfile")));
    }
    if (cmdLine.hasOption("beliefworking")) {
      beliefArgs.add("-workingfile");
      beliefArgs.addAll(Arrays.asList(cmdLine.getOptionValues("beliefworking")));
    }
    if (cmdLine.hasOption("beliefepisodic")) {
      beliefArgs.add("-episodicfile");
      beliefArgs.addAll(Arrays.asList(cmdLine.getOptionValues("beliefepisodic")));
    }
    if (cmdLine.hasOption("beliefuniversal")) {
      beliefArgs.add("-universalfile");
      beliefArgs.addAll(Arrays.asList(cmdLine.getOptionValues("beliefuniversal")));
    }
    if (cmdLine.hasOption("beliefprover")) {
      beliefArgs.add("-prover");
      beliefArgs.add(cmdLine.getOptionValue("beliefprover"));
    }
    if (cmdLine.hasOption("beliefdisk")) {
      beliefArgs.add("-disk");
      beliefArgs.add(cmdLine.getOptionValue("beliefdisk"));
    }
    if (cmdLine.hasOption("beliefg")) {
      beliefArgs.add("-g");
    }
    if (cmdLine.hasOption("beliefgroups")) {
      beliefArgs.add("-groups");
      for (String s : cmdLine.getOptionValues("beliefgroups")) {
        beliefArgs.add(s);
      }
    }
  }

  @Override
  protected void init() {
    // GM must be initialized first, before DB
    initializeGM();
    initializeDB();
    notifier = new GoalManagerNotifier();
    Database.getInstance().addListener(notifier);

    //Load goal priorities
    String filepath = Resources.createFilepath("config/edu/tufts/hrilab/action/manager/priority", priorityFile);
    if (!loadGoalPriorities(filepath)) {
      goalPriorities = new HashMap<>();
      goalPriorities.put("default", new PriorityInfo(1L, PriorityTier.NORMAL));
    }

    PerformanceAssessment.setExecutionManager(em, false);

    //TODO:brad: do we need type in the KB name?
    contextConsultant = new ContextConsultant(em.getRootContext(), "context");
    try {
      List<String> consultantGroups = this.getMyGroups();
      consultantGroups.add(contextConsultant.getKBName());
      TRADE.registerAllServices(contextConsultant, consultantGroups);
    } catch (TRADEException e) {
      log.error("exception registering Context Consultant ", e);
    }

    // sleep here to give GM time to populate with primitive actions
    //TODO:brad: do we actually need to sleep?
    Util.Sleep(2000);
    submitInitGoals();
  }

  // Methods calls during construction
  protected void initializeGM() {
    // set BeliefComponent's groups to be the same as this GoalManager's groups
    List<String> myGroups = getMyGroups();
    if (!myGroups.isEmpty()) {
      beliefArgs.add("-groups");
      beliefArgs.addAll(myGroups);
    }
    StateMachine sm = new StateMachine(beliefArgs.toArray(new String[0]));
    RootContext rootContext = new RootContext(initialConstraints, sm);
    em = ExecutionManager.createInstance(executionManagerType, sm, rootContext, myGroups);
    em.configPruningMechanism(useMemoryManager, historyLength);
    em.setActionLearningGuiFlag(displayActionLearningGui);
  }

  private void initializeDB() {
    // Setup the action database to be used by this component
    // This should be done at the component level because this is a
    // component-wide databse, and not simply used by the ExecutionManager
    // Eventually, each component will be responsible for loading its action db
    //
    //TDOD:brad: I don't think the above is still the plan?
    ActionResourceLockLinear.initResources();

    // Add ASL files to database
    Database.getInstance().loadDatabaseFiles(dbFileDir, dbFiles);

    // Add PDDL model to action db and belief
    if (pddlFiles != null) {
      loadPDDLFiles(Resources.createFilepath(pddlFireDir, pddlFiles.getLeft()), Resources.createFilepath(pddlFireDir, pddlFiles.getRight()));
    }

    // Add performance models
    Database.getInstance().loadPerformanceMeasuresFromFile(performanceFileDir, performanceFiles);

    // Add DIARC agents from Belief
    if (Database.getInstance().getDiarcAgents().isEmpty()) {
      log.warn("No DIARC agents found in Belief. Consider adding at least one diarcAgent(X) to you Belief knowledge base.");
    }

    log.debug("DB initialized");
  }

  /**
   * Add goal (in string form from the command line) to list of initial goals to be automatically executed.
   * (e.g., "stand(shafer)" or "fluent_geq(inventory(self, pogo_stick), 1)")
   *
   * @param goalString
   */
  private void addInitGoal(String goalString) {
    if (goalString.contains(".")) {
      log.error("Goal predicate needs to be in either goal(actor,state) or action(actor,arg) form. Ignoring goal: " + goalString);
      return;
    }
    Predicate goalPred = Factory.createPredicate(goalString);
    initGoals.add(goalPred);
  }

  /**
   * Submit the initial goals to be executed.
   */
  private void submitInitGoals() {
    // create goal for all command line goals
    initGoals.forEach(goal -> submitGoal(goal));
  }

  /**
   * Parse PDDL domain and problem files and add the contents to:
   * 1) actions -> action database
   * 2) facts -> belief (e.g., predicates, functions, types, constants, objects, init, etc)
   * 3) goal -> GM initial goals to be executed
   *
   * @param domainFile
   * @param problemFile
   */
  protected void loadPDDLFiles(String domainFile, String problemFile) {
    PddlParser parser = new PddlParser();
    parser.parse(domainFile, problemFile);

    // add all PDDL facts to belief (e.g., predicates, functions, types, constants, objects, init, etc)
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class)).call(void.class, parser.getFacts());
    } catch (TRADEException e) {
      log.error("Error adding PDDL facts to Belief.", e);
    }

    // add all actions to action DB
    parser.getUnbuiltActions().forEach(action -> action.build(true));

    // add goal to init goals
    Symbol actor = Factory.createSymbol("self");
    Predicate goalPred = Factory.createPredicate("goal", actor, parser.getGoalPredicate());
    initGoals.add(goalPred);
  }

  /**
   * Attempts to load goal priority information defined in the provided json file. If any issues occur, priorities
   * are reset to be equal for all goals.
   *
   * @param filepath the path to the file starting from config.action.manager.priority
   * @return boolean indicating whether the priority information was loaded successfully or not
   */
  private boolean loadGoalPriorities(String filepath) {
    Gson gson = new Gson();
    BufferedReader reader;
    try {
      reader = new BufferedReader((new InputStreamReader(ExecutionManager.class.getResourceAsStream(filepath))));
    } catch (NullPointerException ex) {
      log.error("[loadGoalPriorities] Error loading file {}", filepath, ex);
      return false;
    }

    try {
      goalPriorities = gson.fromJson(reader, new TypeToken<Map<String, PriorityInfo>>() {
      }.getType());
    } catch (JsonSyntaxException e) {
      log.error("[loadGoalPriorities] malformed json in file {}", filepath, e);
      return false;
    } catch (JsonIOException e) {
      log.error("[loadGoalPriorities] unable to load json from file {}", filepath, e);
      return false;
    }

    if (!goalPriorities.containsKey("default")) {
      log.warn("[loadGoalPriorities] don't have 'default' entry, setting to 1");
      goalPriorities.put("default", new PriorityInfo(1L, PriorityTier.NORMAL));
    }
    return true;
  }

  /**
   * Get the default priority tier that would be assigned upon submission for the supplied goal predicate
   */
  @TRADEService
  @Action
  public PriorityTier getPriorityTierForGoal(Predicate g) {
    String goalName = g.getName();
    if (goalPriorities.containsKey(goalName)) {
      return goalPriorities.get(g.getName()).getPriorityTier();
    } else {
      return goalPriorities.get("default").getPriorityTier();
    }
  }

  /**
   * Get the default priority value that would be assigned upon submission for the supplied goal predicate
   */
  @TRADEService
  @Action
  public long getPriorityForGoal(Predicate g) {
    String goalName = g.getName();
    if (goalPriorities.containsKey(goalName)) {
      return goalPriorities.get(g.getName()).getPriority();
    } else {
      return goalPriorities.get("default").getPriority();
    }
  }

  /**
   * Submit a goal to be achieved.
   *
   * @param g the goal in goal(actor,state) or action(actor,args) form
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoal(Predicate g) {
    log.debug("[submitGoal]: " + g);

    return submitGoal(g, ExecutionType.ACT);
  }

  /**
   * Submit a goal to be achieved.
   *
   * @param g    the goal in goal(actor,state) or action(actor,args) form
   * @param type action execution type
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoal(Predicate g, ExecutionType type) {
    log.debug("[submitGoal]: " + g + " with execution type: " + type);
    PriorityTier priorityTier = getPriorityTierForGoal(g);
    return submitGoal(g, type, priorityTier);
  }

  /**
   * Submit a goal to be achieved.
   *
   * @param g            the goal in goal(actor,state) or action(actor,args) form
   * @param type         action execution type
   * @param priorityTier priority tier for the submitted goal (overrides default value)
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoal(Predicate g, ExecutionType type, PriorityTier priorityTier) {
    log.debug("[submitGoal]: " + g + " with execution type: " + type + " and priority tier: " + priorityTier);

    long priority = getPriorityForGoal(g);
    if (priorityTier == null || priorityTier == PriorityTier.UNINITIALIZED) {
      priorityTier = getPriorityTierForGoal(g);
    }
    return submitGoal(g, type, priorityTier, priority);
  }

  /**
   * Submit a goal to be achieved.
   *
   * @param g            the goal in goal(actor,state) or action(actor,args) form
   * @param type         action execution type
   * @param priorityTier priority tier for the submitted goal (overrides default value)
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoal(Predicate g, ExecutionType type, Symbol priorityTier) {
    return submitGoal(g, type, PriorityTier.fromString(priorityTier.toString()));
  }


  /**
   * Submit a goal to be achieved.
   *
   * @param g            the goal in goal(actor,state) or action(actor,args) form
   * @param type         action execution type
   * @param priorityTier priority tier for the submitted goal (overrides default value)
   * @param priority     priority value for the submitted goal (overrides default value)
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoal(Predicate g, ExecutionType type, PriorityTier priorityTier, long priority) {
    log.debug("[submitGoal]: " + g + " with execution type: " + type + ", priority: " + priority + ", and priority tier: " + priorityTier);

    Goal goal = new Goal(g);
    goal.addListener(this.notifier);
    goal.setPriority(priority);
    goal.setPriorityTier(priorityTier);
    goal = em.submitGoal(goal, type);
    if (goal != null) {
      return goal.getId();
    }
    return -1;
  }

  /**
   * Submit a goal with a metric for planning
   *
   * @param g      the goal in goal(actor,state) or action(actor,args) form
   * @param metric the metric in maximize(...) or minimize(...) form
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoalWithMetric(Predicate g, Predicate metric) {
    log.debug("[submitGoal]: " + g);
    PriorityTier priorityTier = getPriorityTierForGoal(g);
    long priority = getPriorityForGoal(g);
    Goal goal = new Goal(g);
    goal.addListener(this.notifier);
    goal.setPriorityTier(priorityTier);
    goal.setPriority(priority);
    goal.setMetric(metric);
    goal = em.submitGoal(goal, ExecutionType.ACT);
    if (goal != null) {
      return goal.getId();
    }
    return -1;
  }

  /**
   * Searches pending and active goals to cancel the goal corresponding to the supplied goalId
   */
  @TRADEService
  @Action
  public void cancelGoal(long gid) {
    em.cancelGoal(gid);
  }

  /**
   * Cancels all pending and active goals (other than the listen goal)
   */
  @TRADEService
  @Action
  public void cancelAllCurrentGoals() {
    em.cancelAllCurrentGoals();
  }

  /**
   * Cancels all active goals (other than the listen goal)
   */
  @TRADEService
  @Action
  public void cancelAllActiveGoals() {
    em.cancelAllActiveGoals();
  }

  /**
   * Cancels all pending goals
   */
  @TRADEService
  @Action
  public void cancelAllPendingGoals() {
    em.cancelAllPendingGoals();
  }

  /**
   * Cancels the upcoming pending goal at the supplied index (ordered in descending priority)
   */
  @TRADEService
  @Action
  public boolean cancelPendingGoalByIndex(int index) {
    return em.cancelPendingGoalByIndex(index);
  }


  /**
   * Searches pending and active goals to suspend the goal corresponding to the supplied goalId
   */
  @TRADEService
  @Action
  public void suspendGoal(long gid) {
    em.suspendGoal(gid);
  }

  /**
   * Searches pending and active goals to resume the goal corresponding to the supplied goalId
   */
  @TRADEService
  @Action
  public void resumeGoal(long gid) {
    em.resumeGoal(gid);
  }

//  /**
//   * Get the goals that are currently being pursued by the Execution Manager for a particular actor.
//   * This includes both goals which are actively undergoing execution (have a corresponding Action Interpreter)
//   * and goals which are being considered by the Execution Manager but may not have been passed on to execution yet
//   *
//   * @return a list of predicates representing the goal
//   */
//  @TRADEService
//  @Action
//  public List<Predicate> getCurrentGoals(Symbol actor) {
//    List<Predicate> pg = new ArrayList<>();
//    for (Goal g : em.getCurrentGoals()) {
//      if (g.getActor().equals(actor)) {
//        pg.add(g.getGoalPredicate());
//      }
//    }
//    return pg;
//  }
//
//  /**
//   * Get a list of all pending and active goals.
//   *
//   * TODO: remove this method, or at least make it not a trade service
//   */
//  @TRADEService
//  @Action
//  public List<Goal> getCurrentGoals() {
//    return em.getCurrentGoals();
//  }
//
//  /**
//   * Get a list of all current pending and/or active goals matching the goalPredicate query.
//   *
//   * TODO: remove this method, or at least make it not a trade service
//   *
//   * @return goals with matching goalPredicate, or null if no matching goal found
//   */
//  @Action
//  @TRADEService
//  public List<Goal> getCurrentGoals(Predicate goalPredicate) {
//    return em.getCurrentGoals(goalPredicate);
//  }

  /**
   * Get a copied list of previously executed goals.
   *
   * @return list of past <code>Goal</code>s
   */
  @TRADEService
  @Deprecated(forRemoval = true)
  public List<Predicate> getPastGoals() {
    List<Predicate> result = new ArrayList<>();
    for (Goal g : em.getPastGoals())
      result.add(g.getGoalPredicate());
    return result;
  }

  /**
   * Get a copied list of the goals currently undergoing execution.
   *
   * @return list of active <code>Goal</code>s
   */
  @TRADEService
  @Deprecated(forRemoval = true)
  public List<Predicate> getActiveGoals() {
    List<Predicate> result = new ArrayList<>();
    for (Goal g : em.getActiveGoals())
      result.add(g.getGoalPredicate());
    return result;
  }

  /**
   * Get the goals that are currently being pursued by the Basic Goal Manager for a particular actor.
   *
   * @return a list of predicates representing the goal
   */
  @TRADEService
  @Action
  @Deprecated(forRemoval = true)
  public List<Predicate> getActiveGoals(Symbol actor) {
    List<Predicate> pg = new ArrayList<>();
    for (Goal g : em.getActiveGoals()) {
      if (g.getActor().equals(actor)) {
        pg.add(g.getGoalPredicate());
      }
    }
    return pg;
  }

  /**
   * Get goal metadata.
   *
   * @param goalID the goal id
   * @return a GoalInfo object
   */
  @TRADEService
  public GoalInfo getGoalInfo(long goalID) {
    return em.getGoal(goalID).getInfo();
  }

  /**
   * Get GoalInfo for all (active, current, past) goals.
   *
   * @return
   */
  @TRADEService
  @Deprecated(forRemoval = true)
  public List<GoalInfo> getAllGoals() {
    List<GoalInfo> goals = new ArrayList<>();
    em.getActiveGoals().forEach(g -> goals.add(g.getInfo()));
    em.getCurrentGoals().forEach(g -> goals.add(g.getInfo()));
    em.getPastGoals().forEach(g -> goals.add(g.getInfo()));
    return goals;
  }

  /**
   * Get a list of predicates of all active goals for all agentTeams
   */
  @TRADEService
  @Action
  @Deprecated(forRemoval = true)
  public List<Predicate> getSystemGoalsPredicates() {
    return em.getSystemGoalsPredicates();
  }

  /**
   * Get a list of predicates of all active goals for the supplied agentTeam and all members
   */
  @TRADEService
  @Action
  @Deprecated(forRemoval = true)
  public List<Predicate> getSystemGoalsPredicates(Symbol actor) {
    return em.getSystemGoalsPredicates(actor);
  }

  /**
   * Get a list of predicates of all pending goals
   */
  @TRADEService
  @Action
  @Deprecated(forRemoval = true)
  public List<Predicate> getPendingGoals() {
    return em.getPendingGoalsPredicates();
  }

  /**
   * Get the predicate of the pending goal currently with the highest priority
   */
  @TRADEService
  @Action
  @Deprecated(forRemoval = true)
  public Predicate getNextGoal() {
    return em.getNextGoal().getGoalPredicate();
  }

  /**
   * Get the predicate of the pending goal for a specific agentTeam currently with the highest priority
   */
  @TRADEService
  @Action
  @Deprecated(forRemoval = true)
  public Predicate getNextGoal(Symbol agent) {
    return em.getNextGoal(agent).getGoalPredicate();
  }

  /**
   * Get the goal ID for all goals matching the supplied predicate. Checks current and
   * past goals.
   *
   * @param goalPredicate
   * @return
   */
  @TRADEService
  public List<Long> getGoalIds(Predicate goalPredicate) {
    List<Goal> goals = em.getAllGoals(goalPredicate);
    return goals.stream().map(Goal::getId).collect(Collectors.toList());
  }

  /**
   * Replace with a non-index based equivalent, e.g., getGoalId(actor, SYSTEM)
   * @param actor
   * @param index
   * @return
   */
  @Deprecated(forRemoval = true)
  @TRADEService
  public Long getCurrentGoalId(Symbol actor, int index) {
    Goal goal = em.getCurrentGoal(actor, index);
    if (goal != null) {
      return goal.getId();
    } else {
      return -1L;
    }
  }

  /**
   * Get the GoalStatus of a particular goal.
   *
   * @param gid the goal to check on
   * @return the GoalStatus
   */
  @TRADEService
  @Action
  public GoalStatus getGoalStatus(long gid) {
    return em.getGoalStatus(gid);
  }

  /**
   * Wait on the goal to have terminal goal status.
   *
   * @param gid the goal
   * @return the GoalStatus of the goal
   */
  @TRADEService
  @Action
  public GoalStatus joinOnGoal(long gid) {
    return em.joinOnGoal(gid);
  }

  /**
   * Wait at most millis milliseconds for the goal to have terminal goal status.
   *
   * @param gid    the goal
   * @param millis milliseconds to wait
   * @return the GoalStatus of the goal
   */
  @TRADEService
  @Action
  public GoalStatus joinOnGoal(long gid, long millis) {
    return em.joinOnGoal(gid, millis);
  }

  /**
   * Get the ActionStatus for the action that is/was executed in pursuit of the specified goal.
   *
   * @param gid
   * @return
   */
  @TRADEService
  @Action
  public ActionStatus getActionStatus(long gid) {
    return em.getActionStatus(gid);
  }

  /**
   * Get failure condition for a particular goal.
   *
   * @param gid the goal to check on
   * @return a justification indicating the cause of failure
   */
  @TRADEService
  @Action
  public Justification getGoalFailConditions(long gid) {
    return em.getGoalFailConditions(gid);
  }

  /**
   * Register to be notified of the specified NotificationType.
   *
   * @see edu.tufts.hrilab.action.notification.GoalManagerNotifier for details on expected callback signatures.
   *
   * @param callback TRADE service callback
   * @param type NotificationType
   */
  @TRADEService
  public void registerGoalManagerNotification(TRADEServiceInfo callback, NotificationType type) {
    notifier.registerNotification(callback, type);
  }

  /**
   * Unregister to be notified of the specified NotificationType.
   *
   * @param callback TRADE service callback
   * @param type NotificationType
   */
  @TRADEService
  public void unregisterGoalManagerNotification(TRADEServiceInfo callback, NotificationType type) {
    notifier.unregisterNotification(callback, type);
  }

  /**
   * Retrieves a predicate description of the steps that would be taken to complete the specified goal. This method
   * will call the local action selector to "plan" the top-level steps and generate the steps predicate.
   *
   * @param goalPredicate goal predicate
   * @return predicate of the form: steps(step1, step2, ...) or null
   */
  @TRADEService
  public Predicate getActionSteps(Predicate goalPredicate) {

    // pick action, and get event specs from selected action
    ActionSelector actionSelector = ActionSelector.getInstance();
    Goal goal = new Goal(goalPredicate);
    ParameterizedAction parameterizedAction = actionSelector.selectActionForGoal(goal, initialConstraints, em.getStateMachine());

    // do null check for action
    Predicate actionSteps = null;
    if (parameterizedAction != null) {
      actionSteps = parameterizedAction.getStepsInPredicateForm();
    }
    return actionSteps;
  }

  /**
   * Retrieves the action context steps associated with the given goal and location predicates. This requires a
   * gaol matching the provided goalPredicate to be an existing CURRENT goal.
   *
   * @param goalPredicate the predicate that identifies the goal for which to retrieve context steps
   * @param locationPredicate the predicate that identifies the location of the goal's context tree to evaluate
   * @return a predicate representing the steps in the action context derived from the provided goal and location predicates,
   *         or null if a unique goal cannot be identified
   */
  public Predicate getActionContextSteps(Predicate goalPredicate, Predicate locationPredicate) {

    // get Goal instance from goalPredicate
    List<Goal> goals = em.getCurrentGoals(goalPredicate);
    if (goals.size() != 1) {
      log.error("[getActionContextSteps] Unique goal not found for goalPredicate: " + goalPredicate);
      return null;
    }
    Goal goalInstance = goals.get(0);

    ContextDescription contextDescription = goalInstance.getContextDescription(locationPredicate);
    return contextDescription.getStepsInPredicateForm();
  }

  /**
   * Helper method that filters and returns a list of goals associated with a specific actor and
   * satisfying the given filter criteria.
   *
   * @param actor the symbol representing the actor whose goals are to be filtered
   * @param filter the filter criteria to apply to the list of goals
   * @return a list of goals associated with the specified actor and meeting the filter criteria
   */
  private List<Goal> getFilteredGoals(Symbol actor, GoalFilter filter) {
    List<Goal> goals = getFilteredGoals(filter);
    goals.removeIf(g -> !g.getActor().equals(actor)); // TODO: how to handle type info?
    return goals;
  }

  /**
   * Helper method that filters a list of goals based on the specified goalPredicate and filter criteria.
   *
   * @param goalPredicate a predicate used to evaluate and filter goals.
   * @param filter the filtering criteria to further narrow down the goals.
   * @return a list of filtered goals matching both the predicate and the filter criteria.
   */
  private List<Goal> getFilteredGoals(Predicate goalPredicate, GoalFilter filter) {
    List<Goal> goals = getFilteredGoals(filter);
    goals.removeIf(g -> !g.getGoalPredicate().instanceOf(goalPredicate));
    return goals;
  }

  /**
   * Helper method that filters and retrieves a list of goals based on the provided filter criteria.
   *
   * @param filter the criteria used to filter the goals. The filter determines
   *               the type of goals to be included, such as active, pending,
   *               past, system-defined, or all goals.
   * @return a list of goals that meet the filter criteria. The list may include
   *         active, pending, next, past, system, or all goals depending on the
   *         filter specified.
   */
  private List<Goal> getFilteredGoals(GoalFilter filter) {
    List<Goal> goals = new ArrayList<>();
    switch (filter) {
      case ACTIVE -> goals.addAll(em.getActiveGoals());
      case CURRENT -> {
        goals.addAll(em.getActiveGoals());
        goals.addAll(em.getPendingGoals());
      }
      case PENDING -> goals.addAll(em.getPendingGoals());
      case NEXT -> goals.add(em.getNextGoal());
      case PAST -> goals.addAll(em.getPastGoals());
      case SYSTEM -> goals.addAll(em.getSystemGoals());
      case ALL -> {
        goals.addAll(em.getActiveGoals());
        goals.addAll(em.getPendingGoals());
        goals.addAll(em.getPastGoals());
      }
    }

    return goals;
  }

  /**
   * Retrieves the goal predicate associated with a specific goal ID.
   *
   * @param goalId the unique identifier of the goal
   * @return the goal predicate associated with the specified goal
   */
  @TRADEService
  public Predicate getGoalPredicate(Long goalId) {
    return em.getGoal(goalId).getGoalPredicate();
  }

  /**
   * Retrieves a list of goal predicates associated with a specific actor, filtered by the provided goal filter.
   *
   * @param actor the actor whose goal predicates are to be retrieved
   * @param filter the filter applied to determine which goals are included
   * @return a list of predicates representing the filtered goals of the given actor
   */
  @TRADEService
  public List<Predicate> getGoalPredicatesForActor(Symbol actor, GoalFilter filter) {
    List<Goal> goals = getFilteredGoals(actor, filter);
    List<Predicate> goalPredicates = new ArrayList<>();
    goals.forEach(goal -> goalPredicates.add(goal.getGoalPredicate()));
    return goalPredicates;
  }

  /**
   * Retrieves a list of goal predicates based on a specified goal predicate and filter criteria.
   *
   * @param goalPredicate the predicate representing the goal to match.
   * @param filter the filter criteria to apply when retrieving goal predicates.
   * @return a list of predicates derived from the filtered goals.
   */
  @TRADEService
  public List<Predicate> getGoalPredicates(Predicate goalPredicate, GoalFilter filter) {
    List<Goal> goals = getFilteredGoals(goalPredicate, filter);
    List<Predicate> goalPreds = new ArrayList<>();
    goals.forEach(goal -> goalPreds.add(goal.getGoalPredicate()));
    return goalPreds;
  }

  /**
   * Retrieves a list of goal informations for a specific actor based on the provided filter.
   *
   * @param actor The symbol representing the actor for whom the goal information is to be retrieved.
   * @param filter The filter criteria used to refine the list of goals associated with the actor.
   * @return A list of GoalInfo objects corresponding to the filtered goals of the actor.
   */
  @TRADEService
  public List<GoalInfo> getGoalInfosForActor(Symbol actor, GoalFilter filter) {
    List<Goal> goals = getFilteredGoals(actor, filter);
    List<GoalInfo> goalInfos = new ArrayList<>();
    goals.forEach(goal -> goalInfos.add(goal.getInfo()));
    return goalInfos;
  }

  /**
   * Retrieves a list of GoalInfo objects based on the given goal predicate and filter criteria.
   *
   * @param goalPredicate a Predicate that defines the condition to filter the goals.
   * @param filter a GoalFilter object containing specific filtering details.
   * @return a list of GoalInfo objects that match the given predicate and filter criteria.
   */
  @TRADEService
  public List<GoalInfo> getGoalInfos(Predicate goalPredicate, GoalFilter filter) {
    List<Goal> goals = getFilteredGoals(goalPredicate, filter);
    List<GoalInfo> goalInfos = new ArrayList<>();
    goals.forEach(goal -> goalInfos.add(goal.getInfo()));
    return goalInfos;
  }

  /**
   * Retrieves a list of goal IDs associated with the specified actor based on the provided filter.
   *
   * @param actor  the actor whose goal IDs are to be retrieved
   * @param filter the filter criteria to apply when retrieving goals
   * @return a list of goal IDs that match the specified criteria for the given actor
   */
  @TRADEService
  public List<Long> getGoalIdsForActor(Symbol actor, GoalFilter filter) {
    List<Long> goalIds = new ArrayList<>();
    List<Goal> goals = getFilteredGoals(actor, filter);
    goals.forEach(goal -> goalIds.add(goal.getId()));
    return goalIds;
  }

  /**
   * Retrieves a list of goal IDs based on the provided goal predicate and filter.
   *
   * @param goalPredicate a condition used to filter specific goals
   * @param filter an additional filter object used to refine the goals selection
   * @return a list of IDs of the goals matching the given predicate and filter
   */
  @TRADEService
  public List<Long> getGoalIds(Predicate goalPredicate, GoalFilter filter) {
    List<Long> goalIds = new ArrayList<>();
    List<Goal> goals = getFilteredGoals(goalPredicate, filter);
    goals.forEach(goal -> goalIds.add(goal.getId()));
    return goalIds;
  }

  /**
   * Selects an action for the given goal predicate.
   *
   * @param goalPredicate the predicate representing the goal for which an action needs to be selected
   * @return a parameterized action that satisfies the goal represented by the given predicate
   */
  @TRADEService
  public ParameterizedAction selectActionForGoal(Predicate goalPredicate) {
    Goal goal = new Goal(goalPredicate);
    return ActionSelector.getInstance().selectActionForGoal(goal, initialConstraints, em.getStateMachine());
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////// END: NEW API/////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Utility method mainly used for testing.
   *
   * @param predicate
   */
  public void setState(Predicate predicate) {
    em.getStateMachine().assertBelief(predicate, MemoryLevel.EPISODIC, Factory.createPredicate("setState(gui)"));
  }

  @Override
  public void shutdownComponent() {
    if (this.em != null) {
      // cancel all pending and active goals and wait for them to terminate
      em.getPendingGoals().forEach(goal -> em.cancelGoal(goal.getId()));
      List<Goal> activeGoals = em.getActiveGoals();
      activeGoals.forEach(goal -> em.cancelGoal(goal.getId()));
      activeGoals.forEach(goal -> em.joinOnGoal(goal.getId()));

      try {
        TRADE.deregister(contextConsultant);
        TRADE.deregister(em);
      } catch (Exception e) {
        log.error("[shutdown]", e);
      }
      em.shutdown();
    }
    Database.destroyInstance();
    ActionSelector.destroyInstance();
  }

  @Override
  public String[] getAdapterClassNames() {
    return new String[]{
            GoalViewerAdapter.class.getName(),
            GoalManagerAdapter.class.getName()
    };
  }
}
