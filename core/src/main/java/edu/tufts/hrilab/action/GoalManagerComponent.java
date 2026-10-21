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
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.description.ContextDescription;
import edu.tufts.hrilab.action.execution.*;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.goal.*;
import edu.tufts.hrilab.action.gui.GoalManagerAdapter;
import edu.tufts.hrilab.action.gui.GoalViewerAdapter;
import edu.tufts.hrilab.action.justification.Justification;
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
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;

import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;


/**
 * The Goal Manager (also called the Action Manager or simply “Action”)
 * manages and executes agents' tasks (goals) in DIARC. In addition to simple action
 * execution, the Goal Manager keeps track of the agents' state (using the Belief
 * Component) and verifies that all goals and actions are acceptable and that the
 * conditions for their execution are met. When interfaced with a Planner
 * the Goal Manager can plan as well as avoid and resolve conflicting actions.
 */
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
  private List<String> priorityFiles = new ArrayList<>();
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
  private Map<String, PriorityInfo> goalPriorities = new HashMap<>();
  /**
   * Path to the file containing the set of persisted scheduled goals to load from on startup and write to on shutdown
   * (root will be user.home/.diarc)
   */
  private String scheduledGoalsFile = null;
  /**
   * Path to the file which will have asl for all newly taught actions written to it (can be loaded on startup
   * separately for persistence of taught actions)
   * (root will be user.home/.diarc)
   */
  private String learnedActionsFile = null;

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
    options.add(Option.builder("priorityFile").hasArgs().desc("goal priority information file(s)").build());
    options.add(Option.builder("nomemorymanager").desc("Do not utilize the memory manager in the goal manager to prune old goals").build());
    options.add(Option.builder("historylength").hasArg().argName("time").desc("How long to keep past goals").build());
    options.add(Option.builder("scheduledGoalsFile").hasArg().argName("file").desc("File containing the set of persisted scheduled goals to load from on startup and write to on shutdown").build());
    options.add(Option.builder("learnedActionsFile").hasArg().argName("file").desc("File to write newly learned asl scripts to").build());

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
      priorityFiles.addAll(Arrays.asList(cmdLine.getOptionValues("priorityFile")));
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
    if (cmdLine.hasOption("scheduledGoalsFile")) {
      String filename = cmdLine.getOptionValue("scheduledGoalsFile");
      String homeDir = System.getProperty("user.home");
      String scheduledGoalsDir = Paths.get(homeDir, ".diarc","scheduledGoals").toString();
      scheduledGoalsFile = scheduledGoalsDir + File.separator + filename;
    }
    if (cmdLine.hasOption("learnedActionsFile")) {
      String filename = cmdLine.getOptionValue("learnedActionsFile");
      String homeDir = System.getProperty("user.home");
      String learnedActionsDir = Paths.get(homeDir, ".diarc","learnedActions").toString();
      learnedActionsFile = learnedActionsDir + File.separator + filename;
      try {
        File f = new File(learnedActionsFile);
        f.getParentFile().mkdirs();
        f.createNewFile();
      } catch (IOException e) {
        log.error("[GoalManagerComponent] couldn't create learned actions file in home directory");
        learnedActionsFile = null;
      }
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
    em.setGoalManagerNotifier(notifier);

    //Load goal priorities
    loadGoalPriorities();

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
    if (scheduledGoalsFile != null) {
      loadScheduledGoals(scheduledGoalsFile);
    }
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
    em.setScheduledGoalsFile(scheduledGoalsFile);
    em.setLearnedActionsFile(learnedActionsFile);
  }

  private void initializeDB() {

    // Add ASL files to database
    Database.getInstance().loadDatabaseFiles(dbFileDir, dbFiles);
    em.loadLearnedActions(true);

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

  private void loadScheduledGoals(String filepath) {
    File f = new File(filepath);
    if (!f.exists()) {
      log.warn("[loadScheduledGoals] file not found: " + filepath);
      return;
    }
    try {
      ObjectInputStream is = new ObjectInputStream(new FileInputStream(f));
      List<Predicate> goalPredicates = (List<Predicate>) is.readObject();
      goalPredicates.forEach(this::submitGoal);
      is.close();
    } catch (IOException | ClassNotFoundException e) {
      log.error("[loadScheduledGoals] error loading cached scheduled goals on startup", e);
    }
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
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs").argTypes(Set.class))
              .call(void.class, parser.getFacts());
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
   * Load all priority files and combine them into goalPriorities Map.
   */
  private void loadGoalPriorities() {
    if (priorityFiles.isEmpty()) {
      priorityFiles.add("default.json");
    }

    priorityFiles.forEach(file -> {
      String filepath = Resources.createFilepath("config/edu/tufts/hrilab/action/manager/priority", file);
      goalPriorities.putAll(loadGoalPriorityFile(filepath));
    });

    if (!goalPriorities.containsKey("default")) {
      log.warn("[loadGoalPriorities] don't have 'default' entry, setting to 1");
      goalPriorities.put("default", new PriorityInfo(1L, PriorityTier.NORMAL));
    }
  }

  /**
   * Attempts to load goal priority information defined in the provided json file. If any issues occur, priorities
   * are reset to be equal for all goals.
   *
   * @param filepath the path to the file starting from config.action.manager.priority
   * @return boolean indicating whether the priority information was loaded successfully or not
   */
  private Map<String, PriorityInfo> loadGoalPriorityFile(String filepath) {
    BufferedReader reader;
    try {
      reader = new BufferedReader((new InputStreamReader(ExecutionManager.class.getResourceAsStream(filepath))));
    } catch (NullPointerException ex) {
      log.error("[loadGoalPriorityFile] Error loading file {}", filepath, ex);
      return new HashMap<>();
    }

    Gson gson = new Gson();
    try {
      return gson.fromJson(reader, new TypeToken<Map<String, PriorityInfo>>() {}.getType());
    } catch (JsonSyntaxException e) {
      log.error("[loadGoalPriorityFile] malformed json in file {}", filepath, e);
      return new HashMap<>();
    } catch (JsonIOException e) {
      log.error("[loadGoalPriorityFile] unable to load json from file {}", filepath, e);
      return new HashMap<>();
    }
  }

  /**
   * Get the default priority tier that would be assigned upon submission for the supplied goal predicate
   */
  @TRADEService
  @Action
  public PriorityTier getPriorityTierForGoal(Predicate g) {
    String goalName = g.getName();
    //Persistent goal's priority should be based on inner goal
    if (goalName.equals("persistent") && g.getArgs().size() > 1) {
      goalName = g.get(1).getName();
    }
    if (goalPriorities.containsKey(goalName)) {
      return goalPriorities.get(goalName).getPriorityTier();
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
    //Persistent goal's priority should be based on inner goal
    if (goalName.equals("persistent") && g.getArgs().size() > 1) {
      goalName = g.get(1).getName();
    }
    if (goalPriorities.containsKey(goalName)) {
      return goalPriorities.get(goalName).getPriority();
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
   * @param priority     priority value for the submitted goal (overrides default value)
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoal(Predicate g, ExecutionType type, PriorityTier priorityTier, long priority) {
    return submitGoal(g, type, priorityTier, priority, Factory.createSymbol("unknown"));
  }


  /**
   * Submit a goal to be achieved.
   *
   * @param g            the goal in goal(actor,state) or action(actor,args) form
   * @param type         action execution type
   * @param priorityTier priority tier for the submitted goal (overrides default value)
   * @param priority     priority value for the submitted goal (overrides default value)
   * @param submitter    the entity (currently only the speaker if a dialogue goal) triggering this goal submission
   *
   * @return the goal ID (for checking status)
   */
  @TRADEService
  @Action
  public long submitGoal(Predicate g, ExecutionType type, PriorityTier priorityTier, long priority, Symbol submitter) {
    log.debug("[submitGoal]: " + g + " with execution type: " + type + ", priority: " + priority + ", and priority tier: " + priorityTier);

    Goal goal = Goal.createGoal(g);
    goal.addListener(this.notifier);
    goal.setPriority(priority);
    goal.setPriorityTier(priorityTier);
    goal.setExecutionType(type);
    goal.setSubmitter(submitter);
    goal = em.submitGoal(goal);
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
    Goal goal = Goal.createGoal(g);
    goal.addListener(this.notifier);
    goal.setPriorityTier(priorityTier);
    goal.setPriority(priority);
    goal.setMetric(metric);
    goal = em.submitGoal(goal);
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
  public boolean cancelGoal(long gid) {
    return em.cancelGoal(gid);
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
   * Cancels a subset of all active goals. This method ignores goals with PriorityTier SKIPPENDING and scheduled goals.
   *  This method ignores goals with PriorityTier SKIPPENDING and scheduled goals. In order to accommodate some
   *  practical agent configurations with input methods' addressees, this method can act on the root agent or a
   *  single descendant which has an active goal. If multiple team members have disjoint active goals, this method will
   *  fail.
   * @return the canceled goals' ids
   */
  @TRADEService
  @Action
  public List<Long> cancelSystemGoals() {
    return em.cancelSystemGoals();
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
   * Cancels a subset of active goals for the supplied agentTeam and all of its children. This method ignores goals with
   * PriorityTier SKIPPENDING and scheduled goals. In order to accommodate some practical agent configurations with
   * input methods' addressees, this method can act on the supplied agent or a single descendant which has an active
   * goal. If multiple team members have disjoint active goals, this method will fail.
   * @param agentTeam
   * @return the canceled goals' ids
   */
  @TRADEService
  @Action
  public List<Long> cancelSystemGoals(Symbol agentTeam) {
    return em.cancelSystemGoals(agentTeam);
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
   * Cancels the upcoming pending goals at the supplied indices (Predicate in the form and(index1,index2,...))
   */
  @TRADEService
  @Action
  public boolean cancelPendingGoalsByIndices(Predicate indices) {
    return em.cancelPendingGoalsByIndices(indices);
  }

  //TODO: when having the time, implement this method body in asl and remove this
  /**
   * Cancels scheduled goals at the supplied indices (Predicate in the form and(index1,index2,...))
   */
  @TRADEService
  @Action
  public boolean cancelScheduledGoalsByIndices(Predicate indices) {
    return em.cancelScheduledGoalsByIndices(indices);
  }

  /**
   * Searches pending and active goals to suspend the goal corresponding to the supplied goalId
   */
  @TRADEService
  @Action
  public boolean suspendGoal(long gid) {
    return em.suspendGoal(gid);
  }

  /**
   * Suspends a subset of the root agentTeam's active goals. This method ignores goals with PriorityTier SKIPPENDING and
   * scheduled goals. In order to accommodate some practical agent configurations with input methods' addressees,
   * this method can act on the root agent or a single descendant which has an active goal. If multiple team members
   * have disjoint active goals, this method will fail.
   * @return the suspended goals' ids
   */
  @Action
  @TRADEService
  public List<Long> suspendSystemGoals() {
    return em.suspendSystemGoals();
  }

  /**
   * Suspends a subset of an agentTeam's active goals. This method ignores goals with PriorityTier SKIPPENDING and
   * scheduled goals. In order to accommodate some practical agent configurations with input methods' addressees,
   * this method can act on the supplied agent or a single descendant which has an active goal. If multiple team members
   * have disjoint active goals, this method will fail.
   * @param agentTeam
   * @return the suspended goals' ids
   */
  @Action
  @TRADEService
  public List<Long> suspendSystemGoals(Symbol agentTeam) {
    return em.suspendSystemGoals(agentTeam);
  }

  /**
   * Searches pending and active goals to resume the goal corresponding to the supplied goalId
   */
  @TRADEService
  @Action
  public boolean resumeGoal(long gid) {
    return em.resumeGoal(gid);
  }

  /**
   * Resumes a subset of all root agentTeam's active goals. This method ignores goals with PriorityTier SKIPPENDING
   * and scheduled goals. In order to accommodate some practical agent configurations with input methods'
   * addressees, this method can act on the root agent or a single descendant which has an active goal. If multiple
   * team members have disjoint active goals, this method will fail.
   * @return the resumed goals' ids
   */
  @Action
  @TRADEService
  public List<Long> resumeSystemGoals() {
    return em.resumeSystemGoals();
  }

  /**
   * Resumes a subset of suspended goals for the supplied agentTeam. This method ignores goals with PriorityTier
   * SKIPPENDING and scheduled goals. In order to accommodate some practical agent configurations with input methods'
   * addressees, this method can act on the supplied agent or a single descendant which has an active goal. If multiple
   * team members have disjoint active goals, this method will fail.
   * @param agentTeam
   * @return the resumed goals' ids
   */
  @Action
  @TRADEService
  public List<Long> resumeSystemGoals(Symbol agentTeam) {
    return em.resumeSystemGoals(agentTeam);
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
//  /**
//   * Get a list of all current pending and/or active goals matching the goalPredicate query.
//   * @return goals with matching goalPredicate, or null if no matching goal found
//   *
//   * TODO: remove this method. No Goal instances should be passed in TRADE services.
//   */
//  @TRADEService
//  public List<Goal> getMatchingCurrentGoals(Predicate queryGoal) {
//    return em.getCurrentGoals(queryGoal);
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
    for (Goal g : em.getPastGoals()) {
      result.add(g.getGoalPredicate());
    }
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
    for (Goal g : em.getActiveGoals()) {
      result.add(g.getGoalPredicate());
    }
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

  //TODO: Q: should interactions with active goals exclude scheduled goals?
  /**
   * Get all scheduled goals which have either yet to be submitted, or are currently executing.
   *
   * @return a list of predicates representing the goal
   */
  @TRADEService
  public List<Predicate> getScheduledGoals() {
    List<Predicate> result = new ArrayList<>();
    for (ScheduledGoal sg : em.getScheduledGoals()) {
      result.add(sg.getDisplayPredicate());
    }
    return result;
  }

  /**
   * Get all scheduled goals for a specific actor which have either yet to be submitted, or are currently executing.
   *
   * @param agentTeam the actor to filter the scheduled goals list by
   * @return a list of predicates representing the goal
   */
  @TRADEService
  public List<Predicate> getScheduledGoals(Symbol agentTeam) {
    List<Predicate> pg = new ArrayList<>();
    for (ScheduledGoal sg : em.getScheduledGoals()) {
      if (sg.getActor().equals(agentTeam)) {
        pg.add(sg.getDisplayPredicate());
      }
    }
    return pg;
  }

  /**
   * Get the goal ids of scheduled goals which have either yet to be submitted, or are currently executing.
   *
   * @return a list of scheduled goals' ids
   */
  @TRADEService
  public List<Long> getScheduledGoalIds() {
    List<Long> result = new ArrayList<>();
    for (Goal g : em.getScheduledGoals()) {
      result.add(g.getId());
    }
    return result;
  }

  /**
   * Get the goal ids of scheduled goals for a specific actor which have either yet to be submitted, or are currently.
   *
   * @param agentTeam the actor to filter the scheduled goals list by
   * @return a list of scheduled goals' ids
   */
  @TRADEService
  public List<Long> getScheduledGoalIds(Symbol agentTeam) {
    List<Long> pg = new ArrayList<>();
    for (Goal g : em.getScheduledGoals()) {
      if (g.getActor().equals(agentTeam)) {
        pg.add(g.getId());
      }
    }
    return pg;
  }

  /**
   * Update the scheduled goal specified by the goal ID. The update predicate can
   * specify the next execution/start time (e.g., start(dateString), set a new periodicity (e.g., period(1, hour),
   * specify a new end time (e.g., end(datString), or any combination using an "and" predicate.
   * See the ScheduledGoal class for all update predicate options.
   *
   * This method fails if the supplied timing arguments cannot be parsed or the goal is not a scheduled goal.
   *
   * @param goalId id of the goal to modify
   * @param update Predicate representing how to update the scheduled goal
   * @return whether the goal was successfully updated
   */
  @TRADEService
  public boolean updateScheduledGoal(long goalId, Predicate update) {
    return em.updateScheduledGoal(goalId, update);
  }

  /**
   * Get Date of next scheduled goal execution. Returns null if goalId is
   * for a non-scheduled goal, or if scheduled goal will not execute again.
   *
   * @param goalId scheduled goal ID
   * @return Date or null
   */
  @TRADEService
  public Date getNextScheduledExecutionDate(long goalId) {
    return em.getNextScheduledExecutionDate(goalId);
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
    em.getCurrentGoals().forEach(g -> goals.add(g.getInfo()));
    em.getPastGoals().forEach(g -> goals.add(g.getInfo()));
    return goals;
  }

  /**
   * Determine whether the existing goal has higher priority than the supplied priorityTier and priority values
   *  (This method is used speculatively before goal submission to know how to format tentativeAccept responses, it
   *    is assumed that the goal associated with priorityTier/priority will have a higher goalId and therefore lose ties)
   * @param goalId - id of the existing goal being compared
   * @param priorityTier - priorityTier of the speculative goal being compared to
   * @param priority - priority of the speculative goal being compared to
   */
  @TRADEService
  public boolean hasGreaterPriority(long goalId, PriorityTier priorityTier, long priority) {
    return em.hasGreaterPriority(goalId,priorityTier,priority);
  }

  /**
   * Get the goal ID for a goal predicate. Checks current and
   * past goals. If no goal is found, -1 is returned.
   * NOTE: Depending on the ExecutionManager subclass, more than one match may exist and one will be returned at random.
   *
   * @param goal
   * @return
   */
  @TRADEService
  public Long getGoalId(Predicate goal) {
    List<Long> goalIds = getGoalIds(goal);
    if (goalIds == null || goalIds.size() == 0) {
      return -1L;
    } else {
      return goalIds.get(0);
    }
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
   * Get the GoalInfo for all goals matching the supplied predicate. Checks all goal collections.
   *
   * @param goal
   * @return
   */
  @TRADEService
  public List<GoalInfo> getGoalInfos(Predicate goal) {
    List<Goal> goals = em.getAllGoals(goal);
    return goals.stream().map(Goal::getInfo).collect(Collectors.toList());
  }

  /**
   * Get the Goal for a particular goal ID. This checks current and past goals.
   * Returns null if no matching goal ID is found.
   *
   * TODO: remove this method once PA doesn't use GM
   *
   * @param gid goal id.
   * @return the Goal corresponding to the ID.
   */
  @TRADEService
  @Deprecated(forRemoval = true)
  public Goal getGoal(long gid) {
    return em.getGoal(gid);
  }

  @TRADEService
  @Action
  @Deprecated(forRemoval = true)
  public Goal getCurrentGoal(Symbol actorToCheck, int index) {
    return em.getCurrentGoal(actorToCheck, index);
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
   * Get the PriorityTier of a particular goal.
   *
   * @param gid the goal to check on
   * @return the PriorityTier
   */
  @TRADEService
  @Action
  public PriorityTier getGoalPriorityTier(long gid) {
    return em.getGoalPriorityTier(gid);
  }

  /**
   * Get the priority of a particular goal.
   *
   * @param gid the goal to check on
   * @return the priority
   */
  @TRADEService
  @Action
  public long getGoalPriority(long gid) {
    return em.getGoalPriority(gid);
  }

  /**
   * Wait until a particular goal has its goal status updated to any new value.
   *
   * @param gid the goal id to check on
   * @return The GoalStatus after the soonest update occurred. Returns null if the goal is not found or is
   *          already terminated
   */
  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "interruptWaitForNextGoalStatusUpdate(?gid)", onSuspendServiceCall = "interruptWaitForNextGoalStatusUpdate(?gid)")
  public GoalStatus waitForNextGoalStatusUpdate(long gid) {
    return waitForNextGoalStatusUpdate(gid,0);
  }

  /**
   * Wait until a particular goal has its goal status updated to any new value.
   *
   * @param gid the goal id to check on
   * @param timeout max amount of time to wait. 0 == indefinite
   * @return The GoalStatus after the soonest update occurred. Returns null if the goal is not found, the goal is
   *          already terminated, or if the timeout duration elapses without an updated
   */
  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "interruptWaitForNextGoalStatusUpdate(?gid)", onSuspendServiceCall = "interruptWaitForNextGoalStatusUpdate(?gid)")
  public GoalStatus waitForNextGoalStatusUpdate(long gid, long timeout) {
    return em.waitForNextGoalStatusUpdate(gid,timeout);
  }

  /**
   * Wait until a particular goal has its goal status updated to any new value.
   *
   * @param gid the goal id to check on
   * @param timeout max amount of time to wait. 0 == indefinite
   * @param startStatus the initial status of the supplied goal - supply if worried about synchronization between checks and calls to this method
   * @return The GoalStatus after the soonest update occurred. Returns null if the goal is not found, the goal is
   *          already terminated, or if the timeout duration elapses without an updated
   */
  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "interruptWaitForNextGoalStatusUpdate(?gid)", onSuspendServiceCall = "interruptWaitForNextGoalStatusUpdate(?gid)")
  public GoalStatus waitForNextGoalStatusUpdate(long gid, long timeout, GoalStatus startStatus) {
    return em.waitForNextGoalStatusUpdate(gid,timeout, startStatus);
  }

  @TRADEService
  public void interruptWaitForNextGoalStatusUpdate(long gid) {
    em.interruptWaitForNextGoalStatusUpdate(gid);
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

  //TODO: Look at issue in ScheduledGoalManager executeScheduledGoal - solve differently and remove this
  @TRADEService
  public GoalManagerNotifier getGoalManagerNotifier() {
    return notifier;
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
    Goal goal = Goal.createGoal(goalPredicate);
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
   * @param filter the filter criteria to apply to the list of goals
   * @param actor the symbol representing the actor whose goals are to be filtered
   * @return a list of goals associated with the specified actor and meeting the filter criteria
   */
  private List<Goal> getFilteredGoals(GoalFilter filter, Symbol actor) {
    List<Goal> goals = getFilteredGoals(filter);
    goals.removeIf(g -> !g.getActor().equals(actor)); // TODO: how to handle type info?
    return goals;
  }

  /**
   * Helper method that filters a list of goals based on the specified goalPredicate and filter criteria.
   *
   * @param filter the filtering criteria to further narrow down the goals.
   * @param goalPredicate a predicate used to evaluate and filter goals.
   * @return a list of filtered goals matching both the predicate and the filter criteria.
   */
  private List<Goal> getFilteredGoals(GoalFilter filter, Predicate goalPredicate) {
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
  public List<Predicate> getGoalPredicatesForActor(GoalFilter filter, Symbol actor) {
    List<Goal> goals = getFilteredGoals(filter, actor);
    List<Predicate> goalPredicates = new ArrayList<>();
    goals.forEach(goal -> goalPredicates.add(goal.getGoalPredicate()));
    return goalPredicates;
  }

  /**
   * Retrieves a list of goal predicates based on a specified goal predicate and filter criteria.
   *
   * @param filter the filter criteria to apply when retrieving goal predicates.
   * @param goalPredicate the predicate representing the goal to match.
   * @return a list of predicates derived from the filtered goals.
   */
  @TRADEService
  public List<Predicate> getGoalPredicates(GoalFilter filter, Predicate goalPredicate) {
    List<Goal> goals = getFilteredGoals(filter, goalPredicate);
    List<Predicate> goalPreds = new ArrayList<>();
    goals.forEach(goal -> goalPreds.add(goal.getGoalPredicate()));
    return goalPreds;
  }

  /**
   * Retrieves a list of goal predicates based on a specified goal predicate and filter criteria.
   *
   * @param filter the filter criteria to apply when retrieving goal predicates.
   * @return a list of predicates derived from the filtered goals.
   */
  @TRADEService
  public List<Predicate> getGoalPredicates(GoalFilter filter) {
    List<Goal> goals = getFilteredGoals(filter);
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
  public List<GoalInfo> getGoalInfosForActor(GoalFilter filter, Symbol actor) {
    List<Goal> goals = getFilteredGoals(filter, actor);
    List<GoalInfo> goalInfos = new ArrayList<>();
    goals.forEach(goal -> goalInfos.add(goal.getInfo()));
    return goalInfos;
  }

  /**
   * Retrieves a list of GoalInfo objects based on the given goal predicate and filter criteria.
   *
   * @param filter a GoalFilter object containing specific filtering details.
   * @param goalPredicate a Predicate that defines the condition to filter the goals.
   * @return a list of GoalInfo objects that match the given predicate and filter criteria.
   */
  @TRADEService
  public List<GoalInfo> getGoalInfos(GoalFilter filter, Predicate goalPredicate) {
    List<Goal> goals = getFilteredGoals(filter, goalPredicate);
    List<GoalInfo> goalInfos = new ArrayList<>();
    goals.forEach(goal -> goalInfos.add(goal.getInfo()));
    return goalInfos;
  }

  /**
   * Retrieves a list of goal IDs associated with the specified actor based on the provided filter.
   *
   * @param filter the filter criteria to apply when retrieving goals
   * @param actor  the actor whose goal IDs are to be retrieved
   * @return a list of goal IDs that match the specified criteria for the given actor
   */
  @TRADEService
  public List<Long> getGoalIdsForActor(GoalFilter filter, Symbol actor) {
    List<Long> goalIds = new ArrayList<>();
    List<Goal> goals = getFilteredGoals(filter, actor);
    goals.forEach(goal -> goalIds.add(goal.getId()));
    return goalIds;
  }

  /**
   * Retrieves a list of goal IDs based on the provided goal predicate and filter.
   *
   * @param filter an additional filter object used to refine the goals selection
   * @param goalPredicate a condition used to filter specific goals
   * @return a list of IDs of the goals matching the given predicate and filter
   */
  @TRADEService
  public List<Long> getGoalIds(GoalFilter filter, Predicate goalPredicate) {
    List<Long> goalIds = new ArrayList<>();
    List<Goal> goals = getFilteredGoals(filter, goalPredicate);
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
    Goal goal = Goal.createGoal(goalPredicate);
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
