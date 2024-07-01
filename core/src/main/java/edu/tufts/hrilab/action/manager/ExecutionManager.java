/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.Gson;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;
import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionListener;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.PerformanceAssessment;
import edu.tufts.hrilab.action.goal.PendingGoal;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.gui.GoalManagerGUI;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.learning.ActionLearning;
import edu.tufts.hrilab.action.learning.ActionLearningStatus;
import edu.tufts.hrilab.action.priority.PriorityCalculator;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.resource.Resources;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

public class ExecutionManager implements ActionListener {
  private static final Logger log = LoggerFactory.getLogger(ExecutionManager.class);

  /**
   * Optional GUI.
   */
  private GoalManagerGUI gui = null;

  /**
   * How often the memory pruning thread should execute pruning mechanism (in milliseconds).
   */
  private final long pruneCycleTime = 60000;

  /**
   * Length of time to keep pastGoals, terminated context trees, and state (in milliseconds).
   */
  private long historyLength = 60000;

  /**
   * Enables the goal manager use pruning mechanism
   */
  private boolean useMemoryManager = true;

  /**
   * Memory management thread scheduler that executes pruning mechanism.
   */
  private final ScheduledExecutorService memoryManager = Executors.newScheduledThreadPool(1);

  /**
   * Dummy interface in order to pass comparison method as a lambda
   */
  private interface GoalComparator {
    int compareGoalPriority(Goal g1, Goal g2);
  }

  /**
   * Comparison method determining the order goals get placed into the pendingGoals queue.
   */
  protected GoalComparator goalComparator = (g1, g2) -> {
    //1st Decider: priority tier
    int val = g1.getPriorityTier().compareTo(g2.getPriorityTier());

    //2nd Decider: priority value
    if (val == 0) {
      val = Long.compare(g1.getPriority(), g2.getPriority());
    }

    //3rd Decider: Older goal (lower id) is greater
    if (val == 0) {
      val = Long.compare(g2.getId(), g1.getId());
    }
    return val;
  };

  /**
   * Team/Agent hierarchy information (transitive)
   */
  protected Map<Symbol, Set<Symbol>> agentHierarchy;

  /**
   * Root AgentTeam in the hierarchy
   */
  protected Symbol rootAgent;

  /**
   * Tree of AgentTeam Objects - contains all active goals spread throughout the
   *  hierarchy
   */
  protected final Map<Symbol,AgentTeam> agentTeams = new HashMap<>();

  protected final Object resourceLock = new Object();

  protected final Object goalsLock = new Object();

  /**
   * Collection maintaining locks for all 'frozen' AgentTeams in order for the
   * behavior to correctly block until unfrozen
   */
  private final Map<Symbol, Lock> freezeLocks = new HashMap<>();
  private final Map<Symbol, Condition> freezeConditions = new HashMap<>();

  /**
   * A collection of all the goals currently waiting to be executed
   */
  protected final TreeSet<PendingGoal> pendingGoals = new TreeSet<>((pg1, pg2) -> goalComparator.compareGoalPriority(pg1.getGoal(), pg2.getGoal()));
  protected final Lock pendingGoalsLock = new ReentrantLock();

  /**
   * A list of past goals.
   */
  private final Set<Goal> pastGoals = Collections.synchronizedSet(new HashSet());

  /**
   * A map of goal names to priority information (tier and value) to be used in ordering pendingGoals before execution.
   * These are loaded from json files found in config.action.manager.priority
   */
  private Map<String, PriorityInfo> goalPriorities;

  /**
   * Handles thread(s) that execute ActionInterpreters.
   */
  private final ExecutorService executor = Executors.newCachedThreadPool();

  /**
   * The goal manager starts with a rootContext that is the root of all execution.
   * All Contexts will be a child of this one (or child of a new one).
   */
  private RootContext rootContext;

  /**
   * The StateMachine is responsible for recording and updating the current state
   * of the world as it is known to this component.
   */
  private StateMachine sm;

  /**
   * Main entry-point for action learning.
   */
  private ActionLearning actionLearning;

  ////Not currently used anywhere, was used in a subclass of ActionResourceLocks. Implementation may be useful as
  ////  reference for lower level resource constraints
  ///**
  // * Priorities is the object that handles calculating the priority for each action
  // * that are being managed.
  // * Priorities is null by default, meaning priorities are not ever updated.
  // */
  //private PriorityCalculator priorities;

  /**
   * External components to be notified of action and associated step progress
   */
  private List<ActionListener> aiListeners;

  public ExecutionManager() {
  }

  protected void init(StateMachine sm, RootContext rootContext, String priorityFile, Collection<String> groups) {
    this.sm = sm;
    this.rootContext = rootContext;

    aiListeners = new ArrayList<>();

    // instantiate action learning instance and register with TRADE to expose services/actions
    actionLearning = new ActionLearning(sm, rootContext, false);
    actionLearning.registerWithTRADE(groups);

    PerformanceAssessment.setExecutionManager(this, false);

    //TODO:brad: prune based on context size not time
    //start memory management thread to prune every N seconds
    if (useMemoryManager) {
      memoryManager.scheduleAtFixedRate(this::pruneOldData, pruneCycleTime, pruneCycleTime, TimeUnit.MILLISECONDS);
    }

    //Load goal priorities
    String filepath = Resources.createFilepath("config/edu/tufts/hrilab/action/manager/priority", priorityFile);
    if (!loadGoalPriorities(filepath)) {
      goalPriorities = new HashMap<>();
      goalPriorities.put("default", new PriorityInfo(1L, PriorityTier.NORMAL));
    }

    //Get agent/team hierarchy from belief
    try {
      agentHierarchy = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllAgentTeams")).call(Map.class);
      //TODO: Change getAllAgentTeams rather than have this block here?
      //Strip type information in order to more easily match
      // against actors in below functions
      Map<Symbol, Set<Symbol>> untypedAgentHierarchy = new HashMap<>();
      for (Map.Entry<Symbol,Set<Symbol>> agentTeam: agentHierarchy.entrySet()) {
        Symbol teamName = agentTeam.getKey();
        teamName = Factory.createSymbol(teamName.toUntypedString());
        Set<Symbol> teamMembers = agentTeam.getValue();
        Set<Symbol> untypedTeamMembers = new HashSet<>();
        for (Symbol teamMember: teamMembers) {
          untypedTeamMembers.add(Factory.createSymbol(teamMember.toUntypedString()));
        }
        untypedAgentHierarchy.put(teamName, untypedTeamMembers);
      }
      agentHierarchy = untypedAgentHierarchy;

      //Initialize all AgentTeams while populating internal links between lower
      // level teams and agents in the hierarchy
      //The aim of this logic is to set only the direct links between teams and
      //  agents, rather than have it be transitive like how it is represented in belief
      for (Symbol teamName: agentHierarchy.keySet()) {
        //Get AgentTeam for team name or create it if null
        AgentTeam team = agentTeams.get(teamName);
        if (team == null) {
          team = new AgentTeam(teamName);
          agentTeams.put(teamName, team);
        }
        //Get number of children of this AgentTeam
        int numMembers = agentHierarchy.get(teamName).size();
        //The AgentTeam with the most children is the root agent
        if (rootAgent == null || agentHierarchy.get(rootAgent).size() < numMembers) {
          rootAgent = teamName;
        }
        //Cycle through children to set internal links
        for (Symbol memberName : agentHierarchy.get(teamName)) {
          if (memberName.getName().equals(teamName.getName())) {
            continue;
          }
          //Create child agentTeam if it doesn't exist
          AgentTeam member = agentTeams.get(memberName.getName());
          if (member == null) {
            member = new AgentTeam(memberName);
            agentTeams.put(memberName, member);
          }
          //Set the above team as this AgentTeam's parent if it has less
          // children than the existing notion of the parent (want most immediate parent)
          AgentTeam existingParent = member.getParentTeam();
          if (existingParent == null || agentHierarchy.get(existingParent.getName()).size() > numMembers) {
            member.addParentTeam(team);
            team.addMember(member);
          }
        }
      }
    } catch (TRADEException e) {
      log.error("[init] error making TRADE call to getAllAgentTeams", e);
    } catch (Exception e) {
      log.error("[init] Agent hierarchy is not defined properly", e);
    }
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
   * @param instanceType ExecutionManager class or subclass to be used
   * @param sm           state machine for root context
   * @param rootContext  base context at the root of all execution
   * @param priorityFile the path to the file starting from config.action.manager.priority containing goal priority information
   * @param groups DIARC group constraints used to register this EM and its class instances that are registered with TRADE (e.g., ActionLearning)
   * @return ExecutionManager instance
   */
  static public ExecutionManager createInstance(Class<ExecutionManager> instanceType, StateMachine sm, RootContext rootContext, String priorityFile, Collection<String> groups) {
    ExecutionManager instance = null;
    try {
      Class[] cArgs = new Class[]{};
      Constructor<? extends ExecutionManager> c = instanceType.getDeclaredConstructor(cArgs);
      instance = c.newInstance();
      instance.init(sm, rootContext, priorityFile, groups);
    } catch (InstantiationException | IllegalAccessException | NoSuchMethodException | InvocationTargetException exception) {
      log.error("couldn't instantiate execution manager " + instanceType, exception);
    }

    try {
      TRADE.registerAllServices(instance, groups);
    } catch (TRADEException e) {
      log.error("Exception registering ExecutionManager with TRADE", e);
    }

    return instance;
  }

  /**
   * Returns the set of AgentTeam(names) whose execution may be affected by a
   * change in task for the supplied AgentTeam. Practically, this returns the
   * supplied agentTeam, its children, and all direct ancestors in the hierarchy
   */
  protected Set<Symbol> getRelevantAgents(Symbol agentTeam) {
    agentTeam = getUntypedSymbol(agentTeam);
    //Gather names of the AgentTeam and all descendants
    Set<Symbol> agentList = getDescendants(agentTeam);
    //Gather names of all direct ancestors
    AgentTeam parent;
    try {
      parent = agentTeams.get(agentTeam).getParentTeam();
    } catch (NullPointerException e) {
      log.error("[getRelevantAgents] agentTeam not found for name {}", agentTeam, e);
      return agentList;
    }
    while (parent != null) {
      agentList.add(parent.getName());
      parent = parent.getParentTeam();
    }
    return agentList;
  }

  /**
   * Returns the names of the supplied agentTeam and all its children in the
   * hierarchy
   */
  protected Set<Symbol> getDescendants(Symbol agentTeam) {
    agentTeam = getUntypedSymbol(agentTeam);

    //Get self and all descendants from hierarchy
    Set<Symbol> agentList = agentHierarchy.get(agentTeam);
    //Agent is a leaf, add only itself
    if (agentList == null) {
      agentList = new HashSet<>();
      agentList.add(agentTeam);
    }
    return agentList;
  }

  ////TODO: Extend definition of resources past just whole agents/teams
  ///**
  // * Returns the set of resources in the system not currently reserved towards
  // * execution of already active goals
  // */
  //private Set<Resource> getAvailableResources() {
  //  Set<Resource> availableResources = new HashSet<>();
  //  synchronized (resourceLock) {
  //    for (AgentTeam agentTeam : agentTeams.values()) {
  //      for (Symbol resName: agentTeam.getResourceNames()) {
  //        if (agentTeam.getResource(resName).isAvailable()) {
  //          availableResources.add(agentTeam.getResource(resName));
  //        }
  //      }
  //    }
  //    return availableResources;
  //  }
  //}

  /**
   * Returns a boolean indicating whether any goal up to maxIndex in the pending
   * collection has a resource conflict with the supplied set.
   */
  protected boolean resourceConflictInPending(Set<Resource> requiredResources, int maxIndex) {
    synchronized (pendingGoalsLock) {
      Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
      int i = 0;
      while (pendingGoalsIterator.hasNext() && i < maxIndex) {
        PendingGoal pg = pendingGoalsIterator.next();
        //If either goal execution would lock resources that need to be
        // available for the other to be executed, then there is a conflict
        //TODO: Store resources in PendingGoal rather than recomputing each submission?
        if (getRequiredResourcesForGoal(pg.getGoal()).stream().anyMatch(requiredResources::contains)) {
          return true;
        }
      }
      return false;
    }
  }

  /**
   * Activate as many pending goals as possible in order of priority while avoiding resource conflicts
   */
  protected void activateValidPendingGoals() {
    Goal assignedGoal = activateNextValidPendingGoal();
    while (assignedGoal != null) {
      assignedGoal = activateNextValidPendingGoal();
    }
  }

  /**
   * Transfers the highest priority goal from the pending collection which has
   *  all required resources available to active
   * @return the goal which was transferred to active for execution. If no such
   * valid goal existed, returns null.
   */
  protected Goal activateNextValidPendingGoal() {
    synchronized (resourceLock) {
      //iterate through pending goals in order of priority
      synchronized (pendingGoalsLock) {
        Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
        while (pendingGoalsIterator.hasNext()) {
          PendingGoal pg = pendingGoalsIterator.next();
          //Collect all resources which need to be available in order to execute this goal
          Set<Resource> necessaryResources = getRequiredResourcesForGoal(pg.getGoal());
          //If all resources are available, submit goal
          if (necessaryResources.stream().allMatch(Resource::isAvailable)) {
            lockResources(pg.getGoal(), necessaryResources);
            transferGoalToActive(pg.getGoal());
            return pg.getGoal();
          }
        }
        //No valid goals found
        return null;
      }
    }
  }

  //This method doesn't need to exist anymore
  /**
   * Indicates whether the supplied goal consumed any resources during execution. Used on goal completion to determine
   * whether a check should be made for new goals to be submitted as a result of completion
   *
   * @param g
   * @return true if the completion of the supplied goal freed up any resources, false otherwise
   */
  protected boolean consumedResources(Goal g) {
    return !getRequiredResourcesForGoal(g).isEmpty();
  }

  /**
   * Return the set of all currently active goals which are occupying some
   * subset of the supplied resource set.
   */
  protected Set<Goal> getResourceConflictingActiveGoals(Set<Resource> requiredResources) {
    Set<Goal> conflictingGoals = new HashSet<>();

    //for all required resources, if res is unavailable then add holder to set
    for (Resource res: requiredResources) {
      if (!res.isAvailable()) {
        //TODO: handle if we allow other mechanisms to lock resources
        conflictingGoals.add(res.getHolder());
      }
    }

    return conflictingGoals;
  }

  /**
   * Returns the list of resource names which are locked from the supplied list
   */
  protected List<Symbol> getLockedResourceNames(Set<Resource> requiredResources) {//, Set<Resource> heldResources) {
    List<Symbol> lockedResourceNames = new ArrayList<>();
    for (Resource res: requiredResources) {
      if (!res.isAvailable()) {
        lockedResourceNames.add(res.getName());
      }
    }

    return lockedResourceNames;
  }

  //TODO: Dont want to define this per-subclass
  // Issue is the base EM working without actually correctly implementing resources
  //  requires the assumption that every goal uses no resources unless otherwise
  //  defined. Whereas the QueueEM requires the assumption that every goal does
  //  use placeholder agent-wide resources unless otherwise defined
  // Replace this impl with the one in the QueueEM and get rid of the overriden method
  //  when resources are correctly implemented. If we want an EM that does not regard
  //  resources at all, I think that should be its own subclass rather than the base
  /**
   * Returns the set of all Resources required to be available in order to
   * execute the supplied goal.
   */
  protected Set<Resource> getRequiredResourcesForGoal(Goal goal) {
    return new HashSet<>();
  }

  //This was only necessary due to the structure of the initial implementation of update notifications. May be obsolete if
  //  if hooks are changed.
  /**
   * Enum used alongside goal notification updates to indicate whether the goal was added, removed, or already present
   * in the relevant collection
   */
  public enum UpdateType {
    REMOVED,
    ADDED,
    UPDATED
  }

  ////////////////////////////////////////////////
  ////// Start Subclass Overridable Methods //////
  ////////////////////////////////////////////////

  //A goal status is passed in explicitly here rather than taken from the passed in goal because we sometimes trigger
  // this callback while separate AI threads are spun up and the desired status causing the actual event to occur hasn't
  // been set yet.

  /**
   * Callback triggered when an active goal's status has been updated.
   *
   * @param g          the goal whose status has been updated
   * @param status     the new status
   * @param updateType indicates whether the active goal was newly added, already existed, or removed
   */
  protected void onActiveGoalUpdated(Goal g, GoalStatus status, UpdateType updateType) {
    //Update UI
    notifyUIActiveGoalUpdated(g, status, updateType);

    //Assign as many pending goals as possible (in order of priority) with resources freed up by this active goal
    // completing
    if (status.isTerminated() && !pendingGoals.isEmpty() && consumedResources(g)) {
      activateValidPendingGoals();
    }
  }

  /**
   * Callback triggered when a pending goal's status has been updated
   *
   * @param g     the goal whose status has been updated
   * @param index the index of the goal in the pendingGoals collection
   *              updateType indicates whether the active goal was newly added, already existed, or removed
   */
  protected void onPendingGoalUpdated(Goal g, int index, UpdateType updateType) {
    //Update UI
    notifyUIPendingGoalUpdated(g, index, updateType);

    //If a pending goal was newly added, check if it should be forwarded straight to execution or left in the queue
    if (updateType == UpdateType.ADDED) {

      synchronized (resourceLock) {
        //Gather required resources for the added goal
        Set<Resource> necessaryResources = getRequiredResourcesForGoal(g);
        //Gather active goals which are occupying resources necessary for this one
        Set<Goal> conflictingGoals = getResourceConflictingActiveGoals(necessaryResources);
        //Transfer to active immediately if the newly submitted goal:
        //1. Does not conflict with any currently active goal
        if (conflictingGoals.isEmpty()) {
          //2. Does not share relevant resources with any higher priority goal currently in the queue
          if (!resourceConflictInPending(necessaryResources,index)) {
            lockResources(g, necessaryResources);
            transferGoalToActive(g);
          }
        }
        //If there is a conflict with an active goal(s), check if the new one should supersede execution
        else {
          if (shouldSupersede(g, conflictingGoals)) {
            supersedeGoals(g, conflictingGoals, necessaryResources);
          } else {
            handleConflictingLowerPriorityGoal(g, necessaryResources);
          }
        }
      }
    }
  }

  /**
   * Determines what is done to an added pending goal when resources are not
   * available to execute the action.
   * @param g the goal that was added
   * @param necessaryResources the required resources which are unavailable
   */
  protected void handleConflictingLowerPriorityGoal(Goal g, Set<Resource> necessaryResources) {
    //TODO: is this check specific enough? And do we want this?
    //If the goal was superseded due to resource conflicts with a higher priority
    //  goal, allow it to sit in pending and eventually resumed when the prior goal
    //  has completed.
    //This is intentionally misaligning behavior of a newly submitted goal and
    //  a superseded goal. One of the main motivations for this is to allow a
    //  persistent goal to continuously act as a "background goal" rather than
    //  be terminated when a higher priority goal comes (for the base EM, works
    //  as is for the queue EM). Could also look to just handle persistent goals
    //  specially instead
    if (g.getActionInterpreter() != null) {
      return;
    }

    //Default behavior: Terminate the goal with a relevant failure justification
    // if required resources are not available
    List<Symbol> lockedResources = getLockedResourceNames(necessaryResources);
    //TODO: make sure this is sensible and add pragrule
    Justification justification = new ConditionJustification(false, Factory.createPredicate("availableResources", lockedResources));
    g.setFailConditions(justification);
    g.setAsTerminated(GoalStatus.FAILED);
    transferGoalToPastGoals(g);
  }

  //////////////////////////////////////////////
  ////// End Subclass Overridable Methods //////
  //////////////////////////////////////////////

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

  //TODO: handle typing in action scripts better and/or refactor submitGoal method interfaces
  @TRADEService
  @Action
  public long submitGoal(Predicate g, Symbol priorityTierSymbol) {
    log.debug("[submitGoal]: " + g + " with priority tier: " + priorityTierSymbol);

    long priority = getPriorityForGoal(g);
    PriorityTier priorityTier = PriorityTier.fromString(priorityTierSymbol.getName());
    if (priorityTier == null || priorityTier == PriorityTier.UNINITIALIZED) {
      priorityTier = getPriorityTierForGoal(g);
    }

    Goal goal = submitGoal(g, ExecutionType.ACT, priority, priorityTier);
    if (goal != null) {
      return goal.getId();
    }
    return -1;
  }

  //TODO: Standing in as a TRADEService for removed submitGoalDirectly method.
  // Only currently used in ActionLearning ExecuteWhileLearning. Remove these
  // annotations when the action learning pipeline is updated.
  @Deprecated
  @Action
  @TRADEService
  /**
   * Calls {@link #submitGoal(Goal, ExecutionType, long, PriorityTier)} with the default values
   */
  public Goal submitGoal(Goal g) {
    return submitGoal(g, ExecutionType.ACT);
  }

  /**
   * Calls {@link #submitGoal(Goal, ExecutionType, long, PriorityTier)} with the default values
   */
  public Goal submitGoal(Predicate g) {
    Goal goal = new Goal(g);
    return submitGoal(goal, ExecutionType.ACT);
  }

  /**
   * Calls {@link #submitGoal(Goal, ExecutionType, long, PriorityTier)} with the default values
   */
  public Goal submitGoal(Predicate g, ExecutionType executionType) {
    Goal goal = new Goal(g);
    return submitGoal(goal, executionType);
  }

  public Goal submitGoal(Predicate g, Predicate metric) {
    Goal goal = new Goal(g);
    goal.setMetric(metric);
    return submitGoal(goal, ExecutionType.ACT);
  }


  /**
   * Calls {@link #submitGoal(Goal, ExecutionType, long, PriorityTier)} with the default values
   */
  public Goal submitGoal(Goal g, ExecutionType execType) {
    long priority = getPriorityForGoal(g.getPredicate());

    return submitGoal(g, execType, priority);
  }

  /**
   * Calls {@link #submitGoal(Goal, ExecutionType, long, PriorityTier)} with the default values
   */
  public Goal submitGoal(Predicate g, ExecutionType execType, long priority) {
    Goal goal = new Goal(g);
    return submitGoal(goal, execType, priority);
  }

  /**
   * Calls {@link #submitGoal(Goal, ExecutionType, long, PriorityTier)} with the default values
   */
  public Goal submitGoal(Goal g, ExecutionType execType, long priority) {
    PriorityTier priorityTier = getPriorityTierForGoal(g.getPredicate());

    return submitGoal(g, execType, priority, priorityTier);
  }

  /**
   * Calls {@link #submitGoal(Goal, ExecutionType, long, PriorityTier)} with the default values
   */
  public Goal submitGoal(Predicate g, ExecutionType execType, long priority, PriorityTier priorityTier) {
    Goal goal = new Goal(g);
    return submitGoal(goal, execType, priority, priorityTier);
  }

  /**
   * Submits the goal the execution manager using {@link #addPendingGoal(Goal, ExecutionType, long, PriorityTier)}.
   * This goal will be added to the pool of goals under consideration by the execution manager
   *
   * @param g the goal to be added
   * @return The duplicate goal if present, otherwise the newly submitted goal
   */
  public Goal submitGoal(Goal g, ExecutionType execType, long priority, PriorityTier priorityTier) {
    log.debug("[submitGoal] submitting goal " + g + " with exec type " + execType);

    Symbol untypedActor = getUntypedSymbol(g.getActor());
    if (getAgentTeam(untypedActor) == null) {
      log.error("[submitGoal] actor {} for goal {} not found in the agent hierarchy. Not executing. ", untypedActor, g);
      Justification justification = new ConditionJustification(false, Factory.createPredicate("actorInHierarchy", untypedActor));
      g.setFailConditions(justification);
      g.setAsTerminated(GoalStatus.FAILED);
      pastGoals.add(g);
      return g;
    }

    if (actionLearning.getLearningStatus() == ActionLearningStatus.ACTIVE && !actionLearning.shouldIgnore(g)) {
      actionLearning.addGoal(g);
      if (!actionLearning.shouldExecute()) {
        pastGoals.add(g);
      }
      return g;
    }

    addPendingGoal(g, execType, priority, priorityTier);
    return g;
  }

  /**
   * Wait indefinitely for the goal to have terminal goal status.
   *
   * @param gid goal ID
   * @return GoalStatus of the goal
   */
  public GoalStatus joinOnGoal(long gid) {
    log.debug("[joinOnGoal] waiting for goal to terminate. gid: " + gid);

    // first wait for goal to no longer be pending
    joinOnPendingGoal(gid);

    // goal no longer pending -- wait for goal to be in terminal state
    Goal goal = getGoal(gid);

    if (goal == null) {
      log.warn("[joinOnGoal] Requested goal does not exist, returning GoalStatus.UNKNOWN");
      return GoalStatus.UNKNOWN;
    }

    AgentTeam agent = getAgentTeam(goal.getActor());
    if (agent == null) {
      log.error("[joinOnGoal] encountered goal with actor not in the agent hierarchy: {}", goal.getActor());
      return GoalStatus.UNKNOWN;
    }

    Future goalFuture = agent.getGoalFuture(goal);
    try {
      // if goal is a past goal, it won't have a future
      if (goalFuture != null) {
        goalFuture.get();
      }
    } catch (InterruptedException | ExecutionException e) {
      log.error("[joinOnGoal] Error waiting on goal: " + goal, e);
    }

    return goal.getStatus();
  }

  /**
   * Wait (up to specified milliseconds) for the goal to have terminal goal status.
   *
   * @param gid    goal ID
   * @param millis maximum time to wait for terminal status
   * @return GoalStatus of the goal (may not be a terminal status)
   */
  public GoalStatus joinOnGoal(long gid, long millis) {
    log.debug("[joinOnGoal] waiting for goal to terminate (millis). gid: " + gid);

    // first wait for goal to no longer be pending
    long startTime = System.currentTimeMillis();
    if (!joinOnPendingGoal(gid, millis)) {
      // returned while waiting for pending goal to no longer be pending
      return GoalStatus.PENDING;
    }
    long endTime = System.currentTimeMillis();
    millis = millis - (endTime - startTime); // update wait time to account for time already waited

    // goal no longer pending -- wait for goal to be in terminal state
    Goal goal = getGoal(gid);

    if (goal == null) {
      log.warn("[joinOnGoal] Requested goal does not exist, returning GoalStatus.UNKNOWN");
      return GoalStatus.UNKNOWN;
    }

    AgentTeam agent = getAgentTeam(goal.getActor());
    if (agent == null) {
      log.error("[joinOnGoal] encountered goal with actor not in the agent hierarchy: {}", goal.getActor());
      return GoalStatus.UNKNOWN;
    }

    Future goalFuture = agent.getGoalFuture(goal);
    try {
      // if goal is a past goal, it won't have a future
      if (goalFuture != null) {
        goalFuture.get(millis, TimeUnit.MILLISECONDS);
      }
    } catch (InterruptedException | ExecutionException e) {
      log.error("[join] Error waiting on goal: " + goal, e);
    } catch (TimeoutException e) {
      log.trace("[join] Timeout waiting on goal: " + goal + " timeout: " + millis, e);
    }
    return goal.getStatus();
  }

  /**
   * Add the supplied PendingGoal to the pending collection and notify of update
   */
  private void addPendingGoal(PendingGoal pg) {
    synchronized (pendingGoalsLock) {
      pendingGoals.add(pg);

      int index = getIndexOfPendingGoal(pg); //not ideal, see if we want to change UI hook signature
      //Notify of new goal submission
      onPendingGoalUpdated(pg.getGoal(), index, UpdateType.ADDED);
    }
  }

  /**
   * @param newGoal     new goal in question
   * @param activeGoals list of currently active goals to compare against
   * @return a boolean indicating whether the new goal's execution should take priority over all supplied active goals
   */
  protected boolean shouldSupersede(Goal newGoal, Set<Goal> activeGoals) {
    //This shouldn't be possible with current hooks
    if (activeGoals.isEmpty()) {
      return true;
    }
    //Compare priority of new goals and existing goal
    //Taking approach that the single highest priority goal takes precedence, regardless of the number of agents involved
    else {
      for (Goal g : activeGoals) {
        if (goalComparator.compareGoalPriority(newGoal, g) > 0) {
          return true;
        }
      }
      return false;
    }
  }

  /**
   * Interrupt execution of the provided active goals and push them back to the pending collection, replacing them with
   * execution of a new goal.
   *
   * @param newGoal     the new goal to be executed after interruption
   * @param activeGoals a list of currently active goals to be pushed back to pending
   */
  protected void supersedeGoals(Goal newGoal, Set<Goal> activeGoals, Set<Resource> relevantResources) {
    //Send active goals back to pending
    synchronized (pendingGoalsLock) {
      synchronized (goalsLock) {
        //Suspend and remove active goals back to pending first
        List<Future> aiFutures = new ArrayList<>();
        List<GoalStatus> goalStatuses = new ArrayList<>(); //track original status to know whether to resume when returned to active
        for (Goal activeGoal : activeGoals) {
          goalStatuses.add(activeGoal.getStatus());
          if (activeGoal.getStatus() == GoalStatus.ACTIVE) {
            suspendGoal(activeGoal.getId());
          }
          unlockResources(relevantResources);
          aiFutures.add(removeActiveGoal(activeGoal));
        }

        //Set new Goal as active
        lockResources(newGoal, relevantResources);
        transferGoalToActive(newGoal);

        //Add superseded goals back to pending
        int i = 0;
        for (Goal activeGoal : activeGoals) {
          PendingGoal pg = new PendingGoal(activeGoal, ExecutionType.ACT);
          pg.setPreviousAIFuture(aiFutures.get(i));
          pg.setPreviousStatus(goalStatuses.get(i));
          addPendingGoal(pg);
          i++;
        }
      }
    }
  }

  /**
   * Transfer a goal from pending goals to active goals, and start execution of the goal. Note that the goal is not
   * guaranteed to be in the active goals when this method returns. If the goal fails, for instance, the goal
   * will automatically be moved to the past goals.
   *
   * @param goal the goal to transfer and execute
   */
  protected void transferGoalToActive(Goal goal) {
    log.debug("[transferGoalToActive] in method with goal: " + goal);

    synchronized (pendingGoalsLock) {

      PendingGoal pg = removePendingGoal(goal);
      if (pg == null) {
        log.error("[transferGoalToActive] attempting to submit goal which doesn't exist in pending queue");
        return;
      }

      //If an existing goal was superseded, then it will have an associated, in progress action interpreter. In this case
      //  we want to resume execution instead of creating a new AI
      if (goal.getActionInterpreter() != null) {
        log.debug("[transferGoalToActive] transferring pending goal which already has a partially executed AI present. Not creating a new one");
        //Place the goal back into the active collection
        addActiveGoal(goal, pg.getPreviousAIFuture());
        //Automatically resume the goal which was suspended when superseded
        if (goal.getStatus() != GoalStatus.SUSPENDED) {
          log.warn("[transferGoalToActive] encountered goal with a non-null ActionInterpreter and is not suspended");
          onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.ADDED);
        }
        else {
          if (pg.getPreviousStatus() == GoalStatus.ACTIVE) {
            log.debug("[transferGoalToActive] Resuming goal with previous AI");
            goal.resume();
            onActiveGoalUpdated(goal, GoalStatus.ACTIVE, UpdateType.ADDED);
          } else {
            log.debug("[transferGoalToActive] Leaving goal with status {}", goal.getStatus());
            onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.ADDED);
          }
        }
        return;
      }

      //Attempt to begin execution of this goal and notify the PendingGoal condition
      executeGoal(goal, pg.getExecutionType());
      pg.notifyOfNoLongerPending();
    }
  }

  /**
   * Add an active goal to the corresponding AgentTeam
   * @param g The active goal
   * @param future The active goal's associated ActionInterpreter future
   */
  private void addActiveGoal(Goal g, Future future) {
    Symbol actor = g.getActor();
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[addActiveGoal] received goal with actor not found in hierarchy: {}", actor);
      return;
    }
    agentTeam.addGoal(g, future);
  }

  //TODO: If resources/locks are managed externally, then refactor usages to
  //  have checking for availability and grabbing to occur at the same time
  private boolean lockResources(Goal g) {
    Set<Resource> resources = getRequiredResourcesForGoal(g);
    return lockResources(g, resources);
  }

  private boolean lockResources(Goal g, Set<Resource> resources) {
    synchronized (resourceLock) {
        for (Resource res : resources) {
          if (!res.isAvailable()) {
            log.warn("[lockResources] attempting to lock resource which is not available: {}", res.getName());
            return false;
          }
        }

      for (Resource res : resources) {
        res.setHolder(g);
      }
    }
    return true;
  }

  /**
   * Remove an active goal from the corresponding AgentTeam
   * @param g The goal to be removed
   * @return The goal's associated ActionInterpreter future
   */
  private Future removeActiveGoal(Goal g) {
    Symbol actor = g.getActor();
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[removeActiveGoal] received goal with actor not found in hierarchy: {}", actor);
      return null;
    }

    onActiveGoalUpdated(g, g.getStatus(), UpdateType.REMOVED);
    return agentTeam.removeGoal(g);
  }

  private boolean unlockResources(Goal g) {
    Set<Resource> resources = getRequiredResourcesForGoal(g);
    return unlockResources(resources);
  }

  private boolean unlockResources(Set<Resource> resources) {
    synchronized (resourceLock) {
      for (Resource res : resources) {
        res.releaseHolder();
      }
    }
    return true;
  }

  /**
   * Attempts to create an ActionInterpreter and begin execution of the supplied goal, checking action/state constraints
   *
   * @param goal          the goal
   * @param executionType goal type (act, simulation, ...)
   * @return true if goal was added
   */
  protected boolean executeGoal(Goal goal, ExecutionType executionType) {
    log.debug("[executeGoal] Adding goal " + goal + " (" + executionType + ")");

    RootContext root;
    StateMachine stateMachine;

    // Branch off main state machine if simulation
    stateMachine = (executionType == ExecutionType.ACT) ? sm : new StateMachine(sm, false);
    // Get root context
    root = (executionType == ExecutionType.ACT) ? rootContext : rootContext.createSimulationRoot(stateMachine, executionType);

    // Check that goal itself does not violate root constrains
    Justification constraintCheck = root.getConstraints().verifyConstraints(goal.getPredicate(), stateMachine);

    if (constraintCheck.getValue()) {
      // Instantiate new action interpreter in charge of executing this goal.
      ActionInterpreter ai = ActionInterpreter.createInterpreterFromGoal(goal, root, stateMachine, executionType);
      // Attach all external Listeners
      for (ActionListener al : aiListeners) {
        ai.addListener(al);
      }
      // Attach this ExecutionManager as Listener
      ai.addListener(this);

      startActionInterpreter(ai);
      return true;
    } else {
      log.debug("Goal " + goal + " is not permissible.");
      goal.setFailConditions(constraintCheck);
      goal.setAsTerminated(GoalStatus.FAILED);
    }

    log.debug("Failed to add goal!");
    pastGoals.add(goal);
    return false;
  }

  /**
   * Submits the supplied actionInterpreter to this class' ExecutorService while doing associated bookkeeping
   */
  private void startActionInterpreter(ActionInterpreter ai) {
    log.debug("Starting interpreter for goal " + ai.getGoal() + "...");
    Future aiFuture = executor.submit(ai);

    Goal goal = ai.getGoal();
    addActiveGoal(goal, aiFuture);
    onActiveGoalUpdated(goal, GoalStatus.ACTIVE, UpdateType.ADDED);
    log.debug("Added goal " + goal);
    //updatePriorities();
  }

  //Action listener methods

  /**
   * Cleans up references to completed actions/goals.
   * Also resets the slice time now that there is one less action.
   * Remove a script interpreter from the goal manager.
   *
   * @param ai the AI to remove
   */
  @Override
  public void actionComplete(ActionInterpreter ai) {
    Goal goal = ai.getGoal();

    synchronized (goalsLock) {
      transferGoalToPastGoals(goal);
    }
  }

  public void actionStarted(ActionInterpreter ai) {
    // intentionally empty -- subclass can optionally implement
  }

  public void stepComplete(Context step) {
    // intentionally empty -- subclass can optionally implement
  }

  public void stepStarted(Context step) {
    // intentionally empty -- subclass can optionally implement
  }

  /**
   * Searches for the supplied goal in the pending and active goal collections, moving it to pastGoals and performing
   * the relevant bookkeeping if found
   */
  protected void transferGoalToPastGoals(Goal goal) {
    log.debug("[transferGoalToPastGoals] in method with goal: " + goal);

    synchronized (pendingGoalsLock) {
      PendingGoal pg = removePendingGoal(goal);
      if (pg != null) {
        pastGoals.add(goal);
        pg.notifyOfNoLongerPending();
        return;
      }
    }

    synchronized (goalsLock) {
      // goal is an active goal
      if (getActiveGoal(goal.getId()) != null) {
        unlockResources(goal);
        removeActiveGoal(goal);
        pastGoals.add(goal);
        return;
      }
    }

    log.warn("[transferGoalToPastGoals] No matching pending or active goal could be found for: {}", goal);
  }

  /**
   * Wait indefinitely for the matching pending goal to have its condition notified. Returns immediately if the goal is
   * not found or pending.
   */
  protected void joinOnPendingGoal(long gid) {
    PendingGoal pg = getPendingGoal(gid);

    // if goal is pending, wait for it to become active (or cancelled)
    if (pg != null) {
      pg.waitForNoLongerPending();
      log.debug("[joinOnGoal] done waiting for pending goal to be activated");
    }
  }

  /**
   * Wait (up to specified milliseconds) for the matching pending goal to have its condition notified. Returns
   * immediately if the goal is not found or pending.
   *
   * @return true: goal not pending or not found. false: goal found and is still pending
   */
  private boolean joinOnPendingGoal(long gid, long millis) {
    PendingGoal pg = getPendingGoal(gid);

    // if goal is pending, wait for it to become active (or cancelled)
    if (pg != null) {
      log.debug("[joinOnGoal] waiting for pending goal to be activated");
      return pg.waitForNoLongerPending(millis);
    }

    return true; // true == goal not pending
  }

  /**
   * Adds a pending goal with default priority. See {@link #addPendingGoal(Goal, ExecutionType, long, PriorityTier)}
   */
  protected void addPendingGoal(Goal g, ExecutionType execType) {
    long priority;
    String goalName = g.getPredicate().getName();
    if (goalPriorities.containsKey(goalName)) {
      priority = goalPriorities.get(g.getPredicate().getName()).getPriority();
    } else {
      priority = goalPriorities.get("default").getPriority();
    }
    addPendingGoal(g, execType, priority);

  }

  /**
   * Adds a pending goal with default priority tier, but overridden value. See
   * {@link #addPendingGoal(Goal, ExecutionType, long, PriorityTier)}
   */
  protected void addPendingGoal(Goal g, ExecutionType execType, long priority) {
    PriorityTier priorityTier;
    String goalName = g.getPredicate().getName();
    if (goalPriorities.containsKey(goalName)) {
      priorityTier = goalPriorities.get(g.getPredicate().getName()).getPriorityTier();
    } else {
      priorityTier = goalPriorities.get("default").getPriorityTier();
    }
    addPendingGoal(g, execType, priority, priorityTier);
  }

  /**
   * Add the goal to the pool of goals being considered by this execution manager. When and if execution of the provided
   * goal occurs is subject to the ExecutionManager implementation. Generally, higher priority goals will be executed
   * first.
   *
   * @param g            the submitted goal
   * @param execType     the execution type of the submitted goal
   * @param priority     priority value of the provided goal
   * @param priorityTier priority tier of the provided goal
   */
  protected void addPendingGoal(Goal g, ExecutionType execType, long priority, PriorityTier priorityTier) {
    g.setPriority(priority);
    g.setPriorityTier(priorityTier);
    PendingGoal pg = new PendingGoal(g, execType); //PendingGoal is primed to block on waitForNoLongerPending by default
    addPendingGoal(pg);
  }

  /**
   * Remove the matching PendingGoal from the pending collection and notify of update
   */
  private PendingGoal removePendingGoal(Goal g) {
    synchronized (pendingGoalsLock) {
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      int index = 0;
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pendingGoal = pendingGoalIterator.next();
        if (pendingGoal.getGoal().getId() == g.getId()) {
          pendingGoalIterator.remove();
          onPendingGoalUpdated(g, index, UpdateType.REMOVED); //updating signature to remove index removes need for iterator
          return pendingGoal;
        }
        index++;
      }
      return null;
    }
  }

  /**
   * return the index of the supplied PendingGoal in the pending collection (which is ordered by priority). Returns -1
   * if not found.
   */
  private int getIndexOfPendingGoal(PendingGoal pg) {
    //Can't do this because this doesn't give any indication of how ties are broken
    //pendingGoals.tailSet(pg, false).size();

    synchronized (pendingGoalsLock) {
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      int index = 0;
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pendingGoal = pendingGoalIterator.next();
        if (pendingGoal.getGoal().getId() == pg.getGoal().getId()) {
          return index;
        }
        index++;
      }
    }
    return -1;
  }

  /**
   * Search both pending and active goal collections and cancel the goal with the supplied goal id if found
   *
   * @param gid
   * @return boolean indicating whether the goal was found in one of the collections or not
   */
  public boolean cancelGoal(long gid) {
    // if goal to cancel is pending
    PendingGoal pg = getPendingGoal(gid);
    if (pg != null) {
      cancelPendingGoal(pg);
      return true;
    }

    // else if goal to cancel is active
    Goal goal = getActiveGoal(gid);
    if (goal != null) {
      cancelActiveGoal(goal);
      return true;
    }

    return false;
  }

  /**
   * Cancel the supplied pending goal and transfer to past goals
   *
   * @param pg
   */
  private void cancelPendingGoal(PendingGoal pg) {
    // cancel pending goal
    Goal goal = pg.getGoal();
    if (goal.getActionInterpreter() == null) {
      goal.setStatus(GoalStatus.CANCELED);
      transferGoalToPastGoals(goal);
    }
    //If this goal was previously superseded and has an associated AI, it needs to be canceled
    else {
      cancelActiveGoal(goal);
    }
    pg.notifyOfNoLongerPending();

    //TODO: this call can be replaced with something more efficient to only check lower priority goals
    //If going by the logic that there can be resource conflicts between
    // pending goals, then we need to check if any lower priority pending goals
    // can now be executed immediately due to this goal being canceled.
    activateValidPendingGoals();
  }

  /**
   * Cancel the supplied active goal and transfer to past goals
   *
   * @param goal
   */
  private void cancelActiveGoal(Goal goal) {
    log.debug("[cancelActiveGoal] goal: {}", goal.getPredicate());
    goal.cancel();

    // if goal doesn't have an AI, remove it from goals list and add to pastGoals
    // if goal does have an AI, the actionCompleted method will be called to
    // take care of that
    boolean hasAI = goal.getActionInterpreter() == null ? false : true;
    if (!hasAI) {
      transferGoalToPastGoals(goal);
    }
  }

  /**
   * Search all goal collections and suspend the goal with the supplied goal id if found
   *
   * @param gid
   * @return true if the goal was found and had an active status, otherwise false
   */
  public boolean suspendGoal(long gid) {
    Goal g = getGoal(gid);

    if (g == null) {
      log.warn("[suspendGoal] goal is null.");
      return false;
    }
    if (g.getStatus().equals(GoalStatus.PENDING)) {
      log.warn("[suspendGoal] attempting to suspend a pending goal");
      return false;
    } else if (g.getStatus().isTerminated()) {
      log.warn("[suspendGoal] attempting to suspend an already terminated goal");
      return false;
    } else {
      return suspendActiveGoal(g);
    }
  }

  /**
   * Search the active goal collection and suspend the supplied goal if found
   *
   * @param goal
   * @return true if the goal was found, otherwise false
   */
  private boolean suspendActiveGoal(Goal goal) {
    log.debug("[suspendGoal] " + goal.getPredicate());
    if (goal == null) {
      log.warn("[suspendGoal] goal is null.");
      return false;
    } else if (goal.getStatus() == GoalStatus.SUSPENDED) {
      log.warn("[suspendGoal] goal status is already SUSPENDED");
      return false;
    } else {
      goal.suspend();
      onActiveGoalUpdated(goal, GoalStatus.SUSPENDED, UpdateType.UPDATED);
    }
    return true;
  }

  /**
   * Search all goal collections and resume the goal with the supplied goal id if found
   *
   * @param gid
   * @return true if the goal was found and had an active status, otherwise false
   */
  public boolean resumeGoal(long gid) {
    Goal g = getGoal(gid);

    if (g == null) {
      log.warn("[resumeGoal] goal is null.");
      return false;
    }
    if (g.getStatus().equals(GoalStatus.PENDING)) {
      log.warn("[resumeGoal] attempting to resume a pending goal");
      return false;
    } else if (g.getStatus().isTerminated()) {
      log.warn("[resumeGoal] attempting to resume an already terminated goal");
      return false;
    } else {
      return resumeActiveGoal(g);
    }
  }


  /**
   * Search the active goal collection and resume the supplied goal if found.
   *
   * @param goal
   * @return true if the goal was found, otherwise false
   */
  private boolean resumeActiveGoal(Goal goal) {
    log.debug("[resumeGoal] " + goal);
    if (goal == null) {
      log.warn("[resumeGoal] goal is null.");
      return false;
    }

    goal.resume();
    onActiveGoalUpdated(goal, GoalStatus.ACTIVE, UpdateType.UPDATED);
    return true;
  }

  /**
   * @return A copied list of all current pending and active goals
   */
  public List<Goal> getCurrentGoals() {
    List<Goal> currentGoals;
    synchronized (goalsLock) {
      synchronized (pendingGoalsLock) {
        currentGoals = getActiveGoals();
        currentGoals.addAll(getPendingGoals().stream().map(PendingGoal::getGoal).collect(Collectors.toList()));
      }
    }
    return currentGoals;
  }

  /**
   * Get a copied list of the goals currently undergoing execution.
   *
   * @return list of Goals
   */
  public List<Goal> getActiveGoals() {
    Set<Goal> activeGoals = new HashSet<>();
    synchronized (goalsLock) {
      getActiveGoalsHelper(rootAgent, activeGoals);
    }
    return new ArrayList<>(activeGoals);
  }

  //Add current goals of the supplied agent/team and all children
  //If an agent can be a member of multiple teams, this implementation is invalid
  private void getActiveGoalsHelper(Symbol agent, Set<Goal> goals) {
    AgentTeam agentTeam = getAgentTeam(agent);
    goals.addAll(agentTeam.getActiveGoals());
    for (Symbol member: agentTeam.getMemberNames()) {
      getActiveGoalsHelper(member, goals);
    }
  }

  /**
   * Get a copied list of the currently managed pending goals waiting to begin execution.
   *
   * @return list of PendingGoals
   */
  public List<PendingGoal> getPendingGoals() {
    return new ArrayList(pendingGoals);
  }

  /**
   * Get a copied list of the previously executed goals
   *
   * @return list of Goals
   */
  public List<Goal> getPastGoals() {
    return new ArrayList<>(pastGoals);
  }

  /**
   * Get the goal for a particular goal ID. This checks pending, current, and past goals.
   *
   * @param gid goal id.
   * @return the goal corresponding to the ID.
   */
  public Goal getGoal(long gid) {
    Goal g = null;
    PendingGoal pg = getPendingGoal(gid);
    if (pg != null) {
      g = pg.getGoal();
    }
    if (g == null) {
      g = getActiveGoal(gid);
    }
    if (g == null) {
      g = getPastGoal(gid);
    }
    return g;
  }

  /**
   * Get the n-th most recently submitted goal.
   * 0 = most recently started goal,
   * 1 = 2nd most recently started goal, etc.
   *
   * @param actor actor to consider currently running goals for (can be null, meaning all actors)
   * @param index into the currently running goals
   * @return n-th most recently start goal, or null if no n-th goal is being executed
   */
  public Goal getCurrentGoal(Symbol actor, int index) {
    return getGoalHelper(getCurrentGoals(), actor, index);
  }

  /**
   * Get the n-th most recently started, currently running, goal.
   * 0 = most recently started goal,
   * 1 = 2nd most recently started goal, etc.
   *
   * @param actor actor to consider currently running goals for (can be null, meaning all actors)
   * @param index into the currently running goals
   * @return n-th most recently start goal, or null if no n-th goal is being executed
   */
  public Goal getActiveGoal(Symbol actor, int index) {
    return getGoalHelper(getActiveGoals(), actor, index);
  }

  /**
   * Get an active goal
   *
   * @param gid goal id
   * @return the goal
   */
  private Goal getActiveGoal(long gid) {
    synchronized (goalsLock) {
      Goal g;
      for (Symbol agent : agentTeams.keySet()) {
        g = agentTeams.get(agent).getActiveGoal(gid);
        if (g != null) {
          return g;
        }
      }
    }
    return null;
  }

  /**
   * Get a pending goal
   *
   * @param gid goal id
   * @return the goal
   */
  private PendingGoal getPendingGoal(long gid) {
    synchronized (pendingGoalsLock) {
      for (PendingGoal pg : pendingGoals) {
        if (pg.getGoal().getId() == gid) {
          return pg;
        }
      }
    }
    return null;
  }

  /**
   * Get the goal for a particular past goal ID.
   *
   * @param gid goal id.
   * @return the goal corresponding to the ID.
   */
  private Goal getPastGoal(long gid) {
    synchronized (pastGoals) {
      for (Goal g : pastGoals) {
        if (g.getId() == gid) {
          return g;
        }
      }
    }
    return null;
  }

  /**
   * Get a list of the currently running goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  protected List<Goal> getActiveGoals(Goal queryGoal) {
    return getGoalHelper(getActiveGoals(), queryGoal);
  }

  /**
   * Get a list of all past goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  public List<Goal> getPastGoals(Goal queryGoal) {
    return getGoalHelper(getPastGoals(), queryGoal);
  }

  /**
   * Get a list of all current pending and/or active goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  @Action
  @TRADEService
  public List<Goal> getCurrentGoals(Goal queryGoal) {
    return getGoalHelper(getCurrentGoals(), queryGoal);
  }

  /**
   * Get a list of all pending, active, and past goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  public List<Goal> getAllGoals(Goal queryGoal) {
    synchronized (pastGoals) {
      synchronized (pendingGoalsLock) {
        synchronized (goalsLock) {
          List<Goal> goals = getCurrentGoals(queryGoal);
          goals.addAll(getPastGoals(queryGoal));
          return goals;
        }
      }
    }
  }

  /**
   * Query the GoalStatus of a particular goal.
   *
   * @param gid the ID of the goal to check on
   * @return GoalStatus object indicating the status
   */
  public GoalStatus getGoalStatus(long gid) {
    log.trace("enter goalStatus(long gid)");
    Goal goal = getGoal(gid);

    if (goal != null) {
      return goal.getStatus();
    } else {
      return GoalStatus.UNKNOWN;
    }
  }

  /**
   * Query the GoalStatus of a particular goal.
   *
   * @param gid the ID of the goal to check on
   * @return GoalStatus object indicating the status
   */
  public GoalStatus getActiveGoalStatus(long gid) {
    log.trace("enter goalStatus(long gid)");
    Goal goal = getActiveGoal(gid);

    if (goal != null) {
      return goal.getStatus();
    } else {
      return GoalStatus.UNKNOWN;
    }
  }

  /**
   * Query the ActionStatus of the action (i.e., root context for this goal) being executed
   * in pursuit of the specified goal.
   *
   * @param gid the ID of the goal to check on
   * @return ActionStatus enum indicating the status
   */
  public ActionStatus getActionStatus(long gid) {
    log.trace("enter getActionStatus(long gid)");
    Goal goal = getGoal(gid);

    if (goal != null) {
      return goal.getRootContext().getStatus();
    } else {
      return ActionStatus.UNKNOWN;
    }
  }

  /**
   * Get failure condition for a particular goal.
   *
   * @param gid the ID of the goal to check on
   * @return a list of Predicates describing the failure
   */
  public Justification getGoalFailConditions(long gid) {
    log.debug("[getGoalFailConditions(long gid)] method entered.");

    Goal goal = getGoal(gid);

    if (goal != null) {
      return goal.getFailConditions();
    } else {
      return new ConditionJustification(true, Factory.createPredicate("reason(unknown)"));
    }
  }

  private Goal getGoalHelper(List<Goal> currGoals, Symbol actor, int index) {
    // filter goals for actor (if not null)
    if (actor != null) {
      currGoals = currGoals.stream().filter(goal -> (goal.getActor() == null || goal.getActor().equals(actor))).collect(Collectors.toList());
    }

    // sort using goal start time
//        currGoals.sort((a, b) -> Long.compare(b.getStartTime(), a.getStartTime()));
    currGoals.sort((a, b) -> Long.compare(b.getId(), a.getId()));
    log.debug("[getCurrentGoal(" + actor + "," + index + ")] time ordered goals: " + Arrays.toString(currGoals.toArray()));

    //
    if (index < 0) {
      index = currGoals.size() + index;
    }

    Goal returnGoal = null;
    if (index < currGoals.size()) {
      returnGoal = currGoals.get(index);
    } else {
      log.debug("[getCurrentGoal(" + actor + "," + index + ")] no matching goal found.");
    }
    return returnGoal;
  }

  private List<Goal> getGoalHelper(List<Goal> goalCollection, Goal queryGoal) {
    List<Goal> matchingGoals = new ArrayList<>();
    for (Goal currGoal : goalCollection) {
      if (currGoal.getActor().equals(queryGoal.getActor()) && currGoal.getPredicate().instanceOf(queryGoal.getPredicate())) {
        matchingGoals.add(currGoal);
      }
    }
    return matchingGoals;
  }

  //TODO: Reevaluate whether we actually care about such a method at this level
  //      and what this should really do. Currently is essentially just a
  //      blocking call which holds whatever resources we define for this method
  //      (right now hardcoded above to be all resources for an agentTeam).
  //      This will not work for the base EM until resource definitions are actually
  //      implemented (because the patching assumption is being made that goals
  //      do not take up resources unless otherwise defined)
  /**
   * Calls {@link #freeze(Symbol)} for the root AgentTeam in the hierarchy
   */
  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "endFreeze()", onSuspendServiceCall = "endFreeze()")
  public void freeze() {
    freeze(rootAgent);
  }

  /**
   * 'Freezes' a team or agent and blocks until unfrozen. A frozen agent will
   * have all current goals suspended and will not be able to begin execution of
   * any new ones. Similarly, all members of a frozen team will be frozen.
   */
  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "endFreeze(?agent)", onSuspendServiceCall = "endFreeze(?agent)")
  public void freeze(Symbol agent) {
    log.info("freeze {}", agent);
    agent = getUntypedSymbol(agent);
    //Set the AgentTeam and all children as 'frozen'
    AgentTeam agentTeam = getAgentTeam(agent);
    if (agentTeam == null) {
      log.warn("[freeze] supplied agent is not found in the hierarchy: {}", agent);
      return;
    }

    //Doesn't actually do much of anything anymore, could remove. Only used for error reporting.
    agentTeam.freeze();

    //Block until this AgentTeam is unfrozen
    Lock freezeLock;
    Condition freezeLockCondition;
    boolean alreadyFrozen = freezeLocks.containsKey(agent);
    if (alreadyFrozen) {
      log.warn("[freeze] AgentTeam {} is already frozen, joining on existing lock", agent);
      freezeLock = freezeLocks.get(agent);
      freezeLockCondition = freezeConditions.get(agent);
    } else {
      freezeLock = new ReentrantLock();
      freezeLockCondition = freezeLock.newCondition();
      freezeLocks.put(agent, freezeLock);
      freezeConditions.put(agent, freezeLockCondition);
    }

    freezeLock.lock();
    try {
      freezeLockCondition.await();
    } catch (InterruptedException e) {
      log.error("[freeze]", e);
    } finally {
      freezeLock.unlock();
    }

    if (!alreadyFrozen) {
      freezeLocks.remove(agent);
      freezeConditions.remove(agent);
    }
  }

  /**
   * Calls {@link #endFreeze(Symbol)} for self
   */
  @Action
  @TRADEService
  public void endFreeze() {
    endFreeze(rootAgent);
  }

  /**
   * Ends the frozen status for the supplied agent/team
   */
  @Action
  @TRADEService
  public void endFreeze(Symbol agent) {
    log.info("endFreeze {}", agent);
    agent = getUntypedSymbol(agent);
    //Unfreeze the AgentTeam and all children
    AgentTeam agentTeam = getAgentTeam(agent);
    if (agentTeam == null) {
      log.warn("[freeze] supplied agent is not found in the hierarchy: {}", agent);
      return;
    }

    //Unblock freeze behavior for this AgentTeam
    Lock freezeLock = freezeLocks.get(agent);
    Condition freezeCondition = freezeConditions.get(agent);
    if (freezeCondition == null || freezeLock == null) {
      if (agentTeam.isFrozen()) {
        log.warn("[endFreeze] Cannot end freeze for a child of a frozen AgentTeam. The original frozen ancestor of {} must be unfrozen", agent);
      } else {
        log.warn("[endFreeze] Agent {} is not frozen", agent);
      }
      return;
    }

    agentTeam.unfreeze();

    freezeLock.lock();
    try {
      freezeCondition.signalAll();
    } finally {
      freezeLock.unlock();
    }
  }

  //TODO: implement and make configurable similar to skipsQueue
  //      Where should this live?

  /**
   * Call ignoreTentativeAccept if service is present in the system
   */
  @TRADEService
  @Action
  public boolean checkIgnoreTentativeAccept(Predicate goalPred) {
    Collection<TRADEServiceInfo> tsis=TRADE.getAvailableServices(new TRADEServiceConstraints().name("ignoreTentativeAccept").argTypes(Predicate.class));

    for(TRADEServiceInfo tsi: tsis){
      try {
        if(tsi.call(Boolean.class, goalPred)) return true;
      } catch (TRADEException e) {
        log.error("[checkIgnoreTentativeAccept] error calling ignoreTentativeAccept", e);
      }
    }
    return false;
  }

  //GUI
  //TODO: Probably want this to exist as a listener/notification system similar to those in other components
  //  and have any UI components which care about these updates to subscribe as listeners. Going to hold off
  //  until other GUI work is more concrete
  protected void notifyUIActiveGoalUpdated(Goal g, GoalStatus status, UpdateType updateType) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("notifyActiveGoalUpdated").argTypes(Goal.class,UpdateType.class,GoalStatus.class)).call(void.class, g, updateType,  status);
    } catch (TRADEException e) {
      log.debug("[notifyUIActiveGoalUpdated]",e);
    }
  }

  protected void notifyUIPendingGoalUpdated(Goal g, int index, UpdateType updateType) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("notifyPendingGoalUpdated").argTypes(Goal.class,Integer.class,UpdateType.class)).call(void.class, g, index, updateType);
    } catch (TRADEException e) {
      log.debug("[notifyUIPendingGoalUpdated]",e);
    }
  }

  /**
   * Add ActionListener to list to be notified of ActionInterpreter and StepExecution events
   *
   * @param al listener to be added
   */
  public void addAIListener(ActionListener al) {
    aiListeners.add(al);
  }

  //private void updatePriorities() {
  //  // priorities is null by default, meaning priorities are not ever modified
  //  if (priorities != null) {

  //    // must synchronize explicitly while iterating over synchronizedMap
  //    synchronized (goals) {
  //      for (Goal g : goals.keySet()) {
  //        ActionInterpreter ai = g.getActionInterpreter();
  //        if (ai != null) {
  //          ai.updatePriority(priorities);
  //        }
  //      }
  //    }
  //  }
  //}

  protected Symbol getUntypedSymbol(Symbol s) {
    if (s.hasType()) {
      s = Factory.createSymbol(s.toUntypedString());
    }
    return s;
  }

  protected AgentTeam getAgentTeam(Symbol name) {
    name = getUntypedSymbol(name);
    AgentTeam agentTeam = agentTeams.get(name);
    if (agentTeam == null) {
      log.warn("[getAgentTeam] no AgentTeam found for name: {}", name);
    }
    return agentTeam;
  }

  /**
   * configure the pruning mechanism
   *
   * @param pruneData         denotes if the goal manager should prune past goals
   * @param pastHistoryLength denotes length of time to keep past goals
   */
  public void configPruningMechanism(boolean pruneData, long pastHistoryLength) {
    useMemoryManager = pruneData;
    historyLength = pastHistoryLength;
  }

  public void pruneOldData() {
    log.debug("Pruning...");

    // prune old goals (which have references to context tree)
    long pruneTime = System.currentTimeMillis() - historyLength;

    try {
      // must synchronize explicitly while iterating over synchronizedSet
      synchronized (pastGoals) {
        pastGoals.removeIf(pastGoal -> pastGoal.getEndTime() < pruneTime);
      }

      // prune old state
      sm.prune(historyLength);

      // prune old action traces (i.e., context tree)
      rootContext.prune(historyLength);
    } catch (Exception e) {
      log.error("Exception caught while pruning.", e);
    }

    log.debug("...done pruning.");
  }

  /**
   * Display a GUI for this goal manager
   */
  public void showEditor(String path) {
    gui = new GoalManagerGUI(this, path);
  }

  public StateMachine getStateMachine() {
    return sm;
  }

  public RootContext getRootContext() {
    return rootContext;
  }

  /**
   * Shutdown this ExecutionManager. All currently managed goals are canceled, ending the ActionInterpreter.
   * Note: the goals are automagically moved to the pastGoals list by the ActionListener mechanism.
   */
  public void shutdown() {
    log.debug("Shutting down and cancelling all goals...");

    synchronized (resourceLock) {
      synchronized (goalsLock) {
        synchronized (pendingGoalsLock) {
          List<PendingGoal> pendingGoals = getPendingGoals();
          for (PendingGoal g : pendingGoals) {
            cancelGoal(g.getGoal().getId());
          }

          List<Goal> goals = getActiveGoals();
          for (Goal g : goals) {
            cancelGoal(g.getId());
          }
        }
      }
    }

    memoryManager.shutdown();
    sm.shutdown();
    try {
      TRADE.deregister(actionLearning);
    } catch (Exception e) {
      log.error("[shutdown]", e);
    }
    executor.shutdown();
  }

  //TODO: Should this be implemented by each subclass? - probably
  // this is only used in PA right now and explicitly for SIMULATE_PERFORMANCE exec types

  /**
   * Entry point to submit a context tree to be executed
   *
   * @param g       the goal
   * @param context the context tree which will be executed
   * @return the context which will be executed first
   */
  public Context submitGoalContext(Goal g, Context context) {
    ActionInterpreter ai = ActionInterpreter.createInterpreterFromExecutionTree(g, context);
    Context simulationStartStep = ai.getCurrentStep();
    log.debug("Starting step " + simulationStartStep.getSignatureInPredicateForm());
    for (ActionListener al : aiListeners) {
      ai.addListener(al);
    }
    ai.addListener(this); // fix to include BGM
    startActionInterpreter(ai);
    return simulationStartStep;
  }
}
