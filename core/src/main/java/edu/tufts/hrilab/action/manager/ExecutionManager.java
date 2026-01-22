/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.*;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.listener.ActionListener;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.listener.GoalListener;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.learning.ActionLearningStatus;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

public class ExecutionManager implements GoalListener {
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

  private static final Logger log = LoggerFactory.getLogger(ExecutionManager.class);

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
   * Memory management thread schedule that executes pruning mechanism.
   */
  private final ScheduledExecutorService memoryManager = Executors.newSingleThreadScheduledExecutor();

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
   * hierarchy
   */
  protected final Map<Symbol, AgentTeam> agentTeams = new HashMap<>();

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
  private final Set<Goal> pastGoals = Collections.synchronizedSet(new HashSet<>());

  /**
   * Handles thread(s) that execute ActionInterpreters.
   */
  private final ExecutorService executor = Executors.newCachedThreadPool();

  /**
   * Handles currently scheduled goals.
   */
  private ScheduledGoalManager scheduledGoalManager;

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
   * External components to be notified of action execution step progress
   */
  private final List<ActionListener> aiListeners = new ArrayList<>();

  /**
   * Lock for waitForGoalStatus method.
   */
  private final Lock goalStatusUpdateLock = new ReentrantLock();

  /**
   * Lock condition for waitForGoalStatus method.
   */
  private final Condition goalStatusUpdateCondition = goalStatusUpdateLock.newCondition();

  /**
   * Path to the file which will have asl for all newly taught actions written to it (can be loaded on startup
   * separately for persistence of taught actions)
   * (root will be user.home/.diarc)
   */
  private String learnedActionsFile = null;

  private boolean displayActionLearningGui = false;

  //TODO: remove this
  private Set<Long> interruptWaitForStatusUpdateGids = new HashSet<>();

  protected void init(StateMachine sm, RootContext rootContext, Collection<String> groups) {
    this.sm = sm;
    this.rootContext = rootContext;
    this.scheduledGoalManager = new ScheduledGoalManager(this);


    //TODO:brad: prune based on context size not time
    //start memory management thread to prune every N seconds
    if (useMemoryManager) {
      memoryManager.scheduleAtFixedRate(this::pruneOldData, pruneCycleTime, pruneCycleTime, TimeUnit.MILLISECONDS);
    }

    //Get agent/team hierarchy from belief
    try {
      agentHierarchy = TRADE.getAvailableService(new TRADEServiceConstraints().name("getAllAgentTeams")).call(Map.class);
      //TODO: Change getAllAgentTeams rather than have this block here?
      //Strip type information in order to more easily match
      // against actors in below functions
      Map<Symbol, Set<Symbol>> untypedAgentHierarchy = new HashMap<>();
      for (Map.Entry<Symbol, Set<Symbol>> agentTeam : agentHierarchy.entrySet()) {
        Symbol teamName = agentTeam.getKey();
        teamName = Factory.createSymbol(teamName.toUntypedString());
        Set<Symbol> teamMembers = agentTeam.getValue();
        Set<Symbol> untypedTeamMembers = new HashSet<>();
        for (Symbol teamMember : teamMembers) {
          untypedTeamMembers.add(Factory.createSymbol(teamMember.toUntypedString()));
        }
        untypedAgentHierarchy.put(teamName, untypedTeamMembers);
      }
      agentHierarchy = untypedAgentHierarchy;

      //Initialize all AgentTeams while populating internal links between lower
      // level teams and agents in the hierarchy
      //The aim of this logic is to set only the direct links between teams and
      //  agents, rather than have it be transitive like how it is represented in belief
      for (Symbol teamName : agentHierarchy.keySet()) {
        //Get AgentTeam for team name or create it if null
        AgentTeam team = agentTeams.get(teamName);
        if (team == null) {
          team = new AgentTeam(teamName, this);
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
          AgentTeam member = agentTeams.get(memberName);
          if (member == null) {
            member = new AgentTeam(memberName, this);
            agentTeams.put(memberName, member);
          }
          //Set the above team as this AgentTeam's parent if it has less
          // children than the existing notion of the parent (want most immediate parent)
          AgentTeam existingParent = member.getParentTeam();
          if (existingParent == null || agentHierarchy.get(existingParent.getName()).size() > numMembers) {
            member.setParentTeam(team);
            team.addMember(member);
            if (existingParent != null) {
              existingParent.removeMember(member);
            }
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
   * @param instanceType ExecutionManager class or subclass to be used
   * @param sm           state machine for root context
   * @param rootContext  base context at the root of all execution
   * @param groups       DIARC group constraints used to register this EM and its class instances that are registered with TRADE (e.g., ActionLearning)
   * @return ExecutionManager instance
   */
  static public ExecutionManager createInstance(Class<ExecutionManager> instanceType, StateMachine sm, RootContext rootContext, Collection<String> groups) {
    ExecutionManager instance = null;
    try {
      Class[] cArgs = new Class[]{};
      Constructor<? extends ExecutionManager> c = instanceType.getDeclaredConstructor(cArgs);
      instance = c.newInstance();
      instance.init(sm, rootContext, groups);
    } catch (InstantiationException | IllegalAccessException | NoSuchMethodException |
             InvocationTargetException exception) {
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
    agentTeam = agentTeam.toUntyped();
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
    agentTeam = agentTeam.toUntyped();

    //Get self and all descendants from hierarchy
    Set<Symbol> agentList = agentHierarchy.get(agentTeam);
    //Agent is a leaf, add only itself
    if (agentList == null) {
      agentList = new HashSet<>();
      agentList.add(agentTeam);
    }
    return agentList;
  }

  /**
   * Returns a boolean indicating whether any goal up to maxIndex in the pending collection has a resource conflict with
   * the supplied set. A resource conflict occurs between two goals if one goal holds a resource during execution
   * which the other goal either also needs to hold or requires to be available. Two goals with only overlapping
   * required resources can coexist.
   */
  protected boolean resourceConflictInPending(Goal g, int maxIndex) {
    log.trace("[resourceConflictInPending] in method {}", g);
    Set<Resource> requiredResources = getRequiredResourcesForGoal(g);
    Set<Resource> heldResources = getHeldResourcesForGoal(g);
    synchronized (pendingGoalsLock) {
      log.trace("[resourceConflictInPending] have pendingGoalsLock");
      Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
      int i = 0;
      while (pendingGoalsIterator.hasNext() && i < maxIndex) {
        PendingGoal pg = pendingGoalsIterator.next();
        //If either goal execution would lock resources that need to be
        // available for the other to be executed, then there is a conflict
        if (getRequiredResourcesForGoal(pg.getGoal()).stream().anyMatch(requiredResources::contains) || getHeldResourcesForGoal(pg.getGoal()).stream().anyMatch(heldResources::contains)) {
          log.debug("[resourceConflictInPending] found conflicting goal in pending collection");
          return true;
        }
      }
      log.trace("[resourceConflictInPending] no conflicting goal found");
      return false;
    }
  }

  /**
   * Activate as many pending goals as possible in order of priority while avoiding resource conflicts
   */
  protected void activateValidPendingGoals() {
    log.trace("[activateNextPendingGoals] in method");
    int goalIndex = activateNextValidPendingGoal(0);
    while (goalIndex != -1) {
      goalIndex = activateNextValidPendingGoal(goalIndex);
    }
  }

  /**
   * Transfers the highest priority goal from the pending collection which has
   * all required resources available to active
   *
   * @return the goal which was transferred to active for execution. If no such
   * valid goal existed, returns null.
   */
  protected int activateNextValidPendingGoal(int startSearchIndex) {
    log.trace("[activateNextPendingGoal] in method");
    synchronized (pendingGoalsLock) {
      log.trace("[activateNextPendingGoal] have pendingGoals lock");
      synchronized (resourceLock) {
        log.trace("[activateNextPendingGoal] have resourceLock");
        //iterate through pending goals in order of priority
        Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
        int goalIndex = 0;
        while (pendingGoalsIterator.hasNext()) {
          if (goalIndex >= startSearchIndex) {
            PendingGoal pg = pendingGoalsIterator.next();
            //Collect all resources which need to be available in order to execute this goal
            Set<Resource> necessaryResources = getRequiredResourcesForGoal(pg.getGoal());
            //If all resources are available, submit goal
            if (necessaryResources.stream().allMatch(Resource::isAvailable)) {
              log.debug("[activateNextPendingGoal] found valid goal {}, locking resources {}", pg.getGoal(), necessaryResources);
              lockResources(pg.getGoal());
              transferGoalToActive(pg.getGoal());
              return goalIndex;
            }
          }
          goalIndex++;
        }
        //No valid goals found
        log.trace("[activateNextPendingGoal] no valid goal found");
        return -1;
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
    return !getHeldResourcesForGoal(g).isEmpty();
  }

  /**
   * Return the set of all currently active goals which are occupying some
   * subset of the supplied resource set.
   */
  protected Set<Goal> getResourceConflictingActiveGoals(Set<Resource> requiredResources) {
    log.trace("[getResourceConflictingActiveGoals] in method {}", requiredResources);
    Set<Goal> conflictingGoals = new HashSet<>();

    //for all required resources, if res is unavailable then add holder to set
    for (Resource res : requiredResources) {
      if (!res.isAvailable()) {
        //TODO: handle if we allow other mechanisms to lock resources
        conflictingGoals.add(res.getHolder());
      }
    }

    log.debug("[getResourceConflictingActiveGoals] conflicting active goals: {}", conflictingGoals);
    return conflictingGoals;
  }

  /**
   * Returns the list of resource names which are locked from the supplied list
   */
  protected List<Symbol> getLockedResourceNames(Set<Resource> requiredResources) {//, Set<Resource> heldResources) {
    List<Symbol> lockedResourceNames = new ArrayList<>();
    for (Resource res : requiredResources) {
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
  //EW: This method has since been updated to consider each AgentTeam's learning resource, but still not the agent-wide
  //    resource. This has been done after moving ActionLearning from a system-wide context to an AgentTeam specific
  //    context in order to avoid independent goals from stepping on learning.

  /**
   * Returns the set of all Resources required to be available during execution of the supplied goal.
   */
  protected Set<Resource> getRequiredResourcesForGoal(Goal goal) {
    if (goal.getRequiredResources() != null) {
      return goal.getRequiredResources();
    } else {
      Set<Resource> resourceSet = new HashSet<>();

      for (Symbol agent : getDescendants(goal.getActor())) {
        if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
          if (!goal.isLearningGoal()) {
            resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("learning")));
          }
          //Always requires the agent to be unfrozen
          resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("freeze")));
        }
      }
      goal.setRequiredResources(resourceSet);

      return resourceSet;
    }
  }

  /**
   * Returns the set of all Resources that will be held by the supplied goal during its execution.
   */
  protected Set<Resource> getHeldResourcesForGoal(Goal goal) {
    if (goal.getHeldResources() != null) {
      return goal.getHeldResources();
    } else {
      Set<Resource> resourceSet = new HashSet<>();

      for (Symbol agent : getDescendants(goal.getActor())) {
        if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
          if (!goal.isLearningGoal()) {
            //TODO: Not sure if we want to ever teach teaching or teach freeze? If not, get rid of above conditional block
          if (goal.getPredicate().getName().equals("learnAction")) {
            resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("learning")));
            } else if (goal.getPredicate().getName().equals("freeze")) {
              resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("freeze")));
            }
          }
        }
      }
      goal.setHeldResources(resourceSet);

      return resourceSet;
    }
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
    log.trace("[onActiveGoalUpdated] {}, {}, {}", g, status, updateType.name());

    //Assign as many pending goals as possible (in order of priority) with resources freed up by this active goal
    // completing
    if (status.isTerminated() && !pendingGoals.isEmpty() && consumedResources(g)) {
      log.debug("[onActiveGoalUpdated] active goal terminated, searching for pending goals to activate");
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
    log.trace("[onPendingGoalUpdated] {} at index {}", g, index);

    //If a pending goal was newly added, check if it should be forwarded straight to execution or left in the queue
    if (updateType == UpdateType.ADDED) {
      log.trace("[onPendingGoalUpdated] pending goal has been added");
      synchronized (resourceLock) {
        //Gather required resources for the added goal
        log.trace("[onPendingGoalUpdated] have resourceLock");
        //Gather active goals which are occupying resources necessary for this one
        Set<Goal> conflictingGoals = getResourceConflictingActiveGoals(getRequiredResourcesForGoal(g));
        //Transfer to active immediately if the newly submitted goal:
        //1. Does not conflict with any currently active goal
        if (conflictingGoals.isEmpty()) {
          log.trace("[onPendingGoalUpdated] no conflicting active goals");
          //2. Does not share relevant resources with any higher priority goal currently in the queue
          if (!resourceConflictInPending(g, index)) {
            log.debug("[onPendingGoalUpdated] no conflicting goals, transferring to active");
            lockResources(g);
            transferGoalToActive(g);
          } else {
            log.debug("[onPendingGoalUpdated] conflicting higher priority pending goal exists, leaving in pending");
          }
        } else {
          //If there is a conflict with an active goal(s), check if the new one should supersede execution
          log.trace("[onPendingGoalUpdated] conflicting active goals exist");
          if (shouldSupersede(g, conflictingGoals)) {
            log.debug("[onPendingGoalUpdated] superseding conflicting active goals");
            supersedeGoals(g, conflictingGoals);
          } else {
            log.debug("[onPendingGoalUpdated] Submitted goal is lower priority than conflicting active goal");
            handleConflictingLowerPriorityGoal(g);
          }
        }
      }
      log.trace("[onPendingGoalUpdated] releasing resourceLock");
    } else if (updateType == UpdateType.REMOVED) {
      //If going by the logic that there can be resource conflicts between
      // pending goals, then we need to check if any lower priority pending goals
      // can now be executed immediately due to this goal being canceled.
      log.trace("[onPendingGoalUpdated] {} searching for lower priority which may have been unblocked", g);
      activateValidPendingGoals();
    }
  }

  /**
   * Determines what is done to an added pending goal when resources are not
   * available to execute the action.
   *
   * @param g the goal that was added
   */
  protected void handleConflictingLowerPriorityGoal(Goal g) {
    log.trace("[handleConflictingLowerPriorityGoal] {}", g);
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
      log.trace("[handleConflictingLowerPriorityGoal] superseded goal will be left untouched");
      return;
    }

    //Default behavior: Terminate the goal with a relevant failure justification
    // if required resources are not available
    log.warn("[handleConflictingLowerPriorityGoal] setting failure justification due to unavailable resources");
    List<Symbol> lockedResources = getLockedResourceNames(getRequiredResourcesForGoal(g));
    //TODO: make sure this is sensible and add pragrule
    Justification justification = new ConditionJustification(false, Factory.createPredicate("availableResources", lockedResources));
    g.setFailureStatus(GoalStatus.FAILED, justification);
    transferGoalToPastGoals(g);
  }

  //////////////////////////////////////////////
  ////// End Subclass Overridable Methods //////
  //////////////////////////////////////////////

  /**
   * Submits the goal the execution manager using {@link #addPendingGoal(PendingGoal)}.
   * This goal will be added to the pool of goals under consideration by the execution manager. When and if execution of
   * the provided goal occurs is subject to the ExecutionManager implementation. Generally, higher priority goals will
   * be executed first.
   *
   * @param g the goal to be added
   * @return The duplicate goal if present, otherwise the newly submitted goal
   */
  public Goal submitGoal(Goal g) {
    log.info("[submitGoal] submitting goal {} with exec type {}, priority {}, tier {}", g, g.getExecutionType(), g.getPriority(), g.getPriorityTier());

    // register "this" as a goal listener to be notified when goal status changes
    g.addListener(this);

    Symbol untypedActor = g.getActor().toUntyped();
    if (getAgentTeam(untypedActor) == null) {
      log.error("[submitGoal] actor {} for goal {} not found in the agent hierarchy. Not executing. ", untypedActor, g);
      Justification justification = new ConditionJustification(false, Factory.createPredicate("actorInHierarchy", untypedActor));
      g.setFailureStatus(GoalStatus.FAILED, justification);
      pastGoals.add(g);
      return g;
    }

    if (g.getScheduledState() == 1) {
      // check if goal is a scheduled goal, and handle
      scheduledGoalManager.executeScheduledGoal((ScheduledGoal) g);
      return g;
    } else if (g.getScheduledState() == 0) {
      //TODO: Better way to designate whether a goal is meant for learning or execution. This should be determined
      //      based on the listener of the utterance and handled appropriately. Currently if the agentTeam or any of its
      //      ancestors are learning, it is assumed this goal is meant for that agentTeam's learning. Any goal for an
      //      agentTeam whose descendent is currently learning will result in a resource conflict
      //TODO: handle the above mentioned resource conflict by asking the user. Prevent unintentional state changes which
      //      would break learning if executing while learning.
      //If actor or ancestor AgentTeam currently learning, hand off to relevant ActionLearning instance.
      if (handOffToLearning(g)) {
        return g;
      }
    }
    //else g.getScheduledState == 2 -> child of ScheduledGoal, should not interact with action learning

    // else, a "normal" goal submission
    addPendingGoal(new PendingGoal(g));
    return g;
  }

  /**
   * Get Date of next scheduled goal execution. Returns null if goalId is
   * for a non-scheduled goal, or if scheduled goal will not execute again.
   *
   * @param goalId scheduled goal ID
   * @return Date or null
   */
  public Date getNextScheduledExecutionDate(long goalId) {
    return scheduledGoalManager.getNextScheduledExecutionDate(goalId);
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

    if (goal.isScheduled()) {
      scheduledGoalManager.joinOnScheduledGoal(gid);
      return goal.getStatus();
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

    log.trace("[joinOnGoal] returning for gid {}", gid);
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
      log.trace("[joinOnGoal] timed out for gid {}", gid);
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

    if (goal.isScheduled()) {
      scheduledGoalManager.joinOnScheduledGoal(gid, millis);
      return goal.getStatus();
    }

    Future goalFuture = agent.getGoalFuture(goal);
    try {
      // if goal is a past goal, it won't have a future
      if (goalFuture != null) {
        goalFuture.get(millis, TimeUnit.MILLISECONDS);
      }
    } catch (InterruptedException | ExecutionException e) {
      log.error("[joinOnGoal] Error waiting on goal: " + goal, e);
    } catch (TimeoutException e) {
      log.debug("[joinOnGoal] Timeout waiting on goal: " + goal + " timeout: " + millis, e);
    }
    log.trace("[joinOnGoal] returning for gid {}", gid);
    return goal.getStatus();
  }

  /**
   * Add the supplied PendingGoal to the pending collection and notify of update
   */
  private void addPendingGoal(PendingGoal pg) {
    log.trace("[addPendingGoal] in method with goal {}", pg.getGoal().getId());
    synchronized (pendingGoalsLock) {
      log.trace("[addPendingGoal] have pendingGoalsLock");
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
    log.trace("[shouldSupersede] in method with goals {}, {}", newGoal, activeGoals);
    //This shouldn't be possible with current hooks
    if (activeGoals.isEmpty()) {
      log.warn("[shouldSupersede] method called with empty active goals set");
      return true;
    }
    //Compare priority of new goals and existing goal
    //Taking approach that the single highest priority goal takes precedence, regardless of the number of agents involved
    else {
      for (Goal g : activeGoals) {
        if (goalComparator.compareGoalPriority(newGoal, g) <= 0) {
          log.trace("[shouldSupersede] {} has lower priority than an active goal, returning false", newGoal);
          return false;
        }
      }
      log.debug("[shouldSupersede] {} has higher priority than all active goals, returning true", newGoal);
      return true;
    }
  }

  /**
   * Interrupt execution of the provided active goals and push them back to the pending collection, replacing them with
   * execution of a new goal.
   *
   * @param newGoal     the new goal to be executed after interruption
   * @param activeGoals a list of currently active goals to be pushed back to pending
   */
  protected void supersedeGoals(Goal newGoal, Set<Goal> activeGoals) {
    log.info("[supersedeGoals] {} superseding {}", newGoal, activeGoals);
    //TODO: wary about concurrency here. Can't lock on both collections around suspension of goals because deadlock can
    //  occur if onInterrupt methods do anything involving interruption of other/subgoals. If this is only called through
    //  onPendingGoalUpdated, are we safe here without worrying about new goals coming in? If interruption behaviors
    //  act on pending goals (which shouldn't really ever happen?), are we still in trouble?
    //Send active goals back to pending
    synchronized (pendingGoalsLock) {
      log.trace("[supersedeGoals] have pendingGoalsLock");
      synchronized (goalsLock) {
        log.trace("[supersedeGoals] have goalsLock");
        //Suspend and remove active goals back to pending first
        List<Goal> supersededGoals = new ArrayList<>();
        List<Future> aiFutures = new ArrayList<>();
        List<GoalStatus> goalStatuses = new ArrayList<>(); //track original status to know whether to resume when returned to active
        //TODO: should to-be-superseded goal interruption occur in parallel threads?
        for (Goal activeGoal : activeGoals) {
          if (!activeGoal.getStatus().isTerminated()) {
            GoalStatus currentStatus = activeGoal.getStatus();
            //TODO: Revisit this - kiosk goals cannot be canceled within this function properly due to held locks and multithreaded
            //      nature of AI interruption stopping execution and disjoint status update listener cleaning up the goal itself.
            //      The goal will be canceled, but not move collections or release resources
            if (activeGoal.getPriorityTier() == PriorityTier.KIOSK) {
              //superseded kiosk goals can be canceled and discarded rather than suspended and added to pending
              if (cancelActiveGoal(activeGoal)) {
                waitForGoalStatus(activeGoal, GoalStatus.CANCELED);
                //Shouldn't be necessary, needed until deadlock issues are fixed
                unlockResources(activeGoal);
              }
            } else {
              if (currentStatus == GoalStatus.ACTIVE) {
                log.debug("[supersedeGoals] suspending active goal {}", activeGoal);
                if (suspendActiveGoal(activeGoal)) {
                  // wait for goal to be suspended
                  waitForGoalStatus(activeGoal, GoalStatus.SUSPENDED);
                }
              }
              log.debug("[supersedeGoals] unlocking resources and removing active goal {}", activeGoal);
              unlockResources(activeGoal);
              Future aiFuture = removeActiveGoal(activeGoal);
              if (aiFuture == null) {
                log.warn("[supersedeGoals] goal to supersede may have terminated on its own...");
              } else {
                supersededGoals.add(activeGoal);
                goalStatuses.add(currentStatus);
                aiFutures.add(aiFuture);
              }
            }
          }
        }

        //Set new Goal as active
        log.debug("[supersedeGoals] setting new goal to active {}", newGoal);
        lockResources(newGoal);
        transferGoalToActive(newGoal);

        //Add superseded goals back to pending
        int i = 0;
        for (Goal activeGoal : supersededGoals) {
          log.debug("[supersedeGoals] returning superseded goal to pending {}", activeGoal);
          PendingGoal pg = new PendingGoal(activeGoal);
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
      log.trace("[transferGoalToActive] have pendingGoalsLock");
      PendingGoal pg = removePendingGoal(goal);
      if (pg == null) {
        log.error("[transferGoalToActive] attempting to submit goal which doesn't exist in pending queue");
        return;
      }

      //If an existing goal was superseded, then it will have an associated, in progress action interpreter. In this case
      //  we want to resume execution instead of creating a new AI
      if (goal.getActionInterpreter() != null) {
        log.debug("[transferGoalToActive] transferring pending goal which already has a partially executed AI present. Not creating a new one");
        //Place the goal back into the active collection and send no-longer-pending notification
        addActiveGoal(goal, pg.getPreviousAIFuture());
        pg.notifyOfNoLongerPending();

        //Automatically resume the goal which was suspended when superseded
        if (goal.getStatus() != GoalStatus.SUSPENDED) {
          log.warn("[transferGoalToActive] encountered goal with a non-null ActionInterpreter and is not suspended");
          onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.ADDED);
        } else {
          if (pg.getPreviousStatus() == GoalStatus.ACTIVE) {
            log.debug("[transferGoalToActive] Resuming goal with previous AI");
            goal.resume();
            onActiveGoalUpdated(goal, GoalStatus.ACTIVE, UpdateType.ADDED);
          } else {
            log.debug("[transferGoalToActive] Leaving goal with current status {}", goal.getStatus());
            onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.ADDED);
          }
        }
        return;
      }

      //Attempt to begin execution of this goal and notify the PendingGoal condition
      executeGoal(goal);
      pg.notifyOfNoLongerPending();
    }
  }

  /**
   * Add an active goal to the corresponding AgentTeam
   *
   * @param g      The active goal
   * @param future The active goal's associated ActionInterpreter future
   */
  private void addActiveGoal(Goal g, Future future) {
    Symbol actor = g.getActor();
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[addActiveGoal] received goal with actor not found in hierarchy: {}", actor);
      return;
    }
    log.debug("[addActiveGoal] adding goal {} to agentTeam {}", g, agentTeam.getName());
    agentTeam.addGoal(g, future);
  }

  //TODO: If resources/locks are managed externally, then refactor usages to
  //  have checking for availability and grabbing to occur at the same time
  private boolean lockResources(Goal g) {
    Set<Resource> resources = getHeldResourcesForGoal(g);
    log.debug("[lockResources] {}, {}", g, resources);
    synchronized (resourceLock) {
      log.trace("[lockResources] have resourceLock");
      for (Resource res : resources) {
        if (!res.isAvailable()) {
          log.warn("[lockResources] attempting to lock resource which is not available: {}", res.getName());
          return false;
        }
      }

      log.trace("[lockResources] locking resources {}", resources);
      for (Resource res : resources) {
        res.setHolder(g);
      }
    }
    log.trace("[lockResources] released resourceLock");
    return true;
  }

  /**
   * Remove an active goal from the corresponding AgentTeam
   *
   * @param g The goal to be removed
   * @return The goal's associated ActionInterpreter future
   */
  private Future removeActiveGoal(Goal g) {
    log.debug("[removeActiveGoal] {}", g);
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
    Set<Resource> resources = getHeldResourcesForGoal(g);
    log.debug("[unlockResources] {}", resources);
    synchronized (resourceLock) {
      log.trace("[unlockResources] unlocking resources {}", resources);
      for (Resource res : resources) {
        if (res.getHolder() != g) {
          //Known race condition exists due to synchronization issues from within supersedeGoals when interacting with kiosk goals
          //Fix issue documented in supersedeGoals
          log.warn("[unlockResources] Goal is attempting to unlock resource that it is not holding");
        } else {
          res.releaseHolder();
        }
      }
    }
    log.trace("[unlockResources] released resourceLock");
    return true;
  }

  /**
   * Attempts to create an ActionInterpreter and begin execution of the supplied goal, checking action/state constraints
   *
   * @param goal          the goal
   * @return true if goal was added
   */
  protected boolean executeGoal(Goal goal) {
    log.debug("[executeGoal] Adding goal " + goal + " (" + goal.getExecutionType() + ")");

    RootContext root;
    StateMachine stateMachine;
    ExecutionType executionType = goal.getExecutionType();

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

      log.trace("[executeGoal] starting action interpreter");
      startActionInterpreter(ai);
      return true;
    } else {
      log.info("[executeGoal] Goal " + goal + " is not permissible.");
      goal.setFailureStatus(GoalStatus.FAILED, constraintCheck);
    }

    log.info("[executeGoal] Failed to add goal! {}", goal);
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
    //updatePriorities();
  }

  //TODO: reimplement this, was in a rush
  //Needs to be more specific to each call
  public void interruptWaitForNextGoalStatusUpdate(long gid) {
    interruptWaitForStatusUpdateGids.add(gid);
    goalStatusUpdateLock.lock();
    try {
      goalStatusUpdateCondition.signalAll();
    } finally {
      goalStatusUpdateLock.unlock();
    }
  }

  /**
   * Wait until a particular goal has its goal status updated to any new value.
   *
   * @param gid goal ID
   * @param timeout time to wait in milliseconds
   * @return The GoalStatus after the soonest update occurred or the timeout duration elapsed without an update
   */
  public GoalStatus waitForNextGoalStatusUpdate(long gid, long timeout) {
    return waitForNextGoalStatusUpdate(gid, timeout, null);
  }
  public GoalStatus waitForNextGoalStatusUpdate(long gid, long timeout, GoalStatus startStatus) {
    Goal goal = getGoal(gid);
    if (goal == null || goal.getStatus().isTerminated()) {
      return null;
    }

    if (startStatus == null) {
      startStatus = goal.getStatus();
    }

    return waitForNextGoalStatusUpdate(goal, timeout, startStatus);
  }

  /**
   * Wait until a particular goal has its goal status updated to any new value.
   *
   * @param goal
   * @param timeout
   * @return The GoalStatus after the soonest update occurred or the timeout duration elapsed without an update
   */
  protected GoalStatus waitForNextGoalStatusUpdate(Goal goal, long timeout, GoalStatus startStatus) {
    goalStatusUpdateLock.lock();
    long startTime = System.currentTimeMillis();
    try {
      while (goal.getStatus() == startStatus && (timeout == 0 || (System.currentTimeMillis() - startTime < timeout))) {
        //TODO: even with this impl, doesn't handle if there are multiple listeners on the same goal
        if (interruptWaitForStatusUpdateGids.contains(goal.getId())) {
          interruptWaitForStatusUpdateGids.remove(goal.getId());
          return null;
        }
        if (timeout == 0) {
          goalStatusUpdateCondition.await();
        } else {
          if (!goalStatusUpdateCondition.await(timeout - (System.currentTimeMillis() - startTime), TimeUnit.SECONDS)) {
            return null;
          }
        }
      }
    } catch (InterruptedException e) {
      log.error("Error waiting on new status for goal {}", goal);
    } finally {
      goalStatusUpdateLock.unlock();
    }
    return goal.getStatus();
  }

  /**
   * Convenience method to wait until a particular goal has a particular goal status. This
   * relies on the GoalListener callback.
   *
   * @param goal
   * @param status
   */
  protected void waitForGoalStatus(Goal goal, GoalStatus status) {
    goalStatusUpdateLock.lock();
    try {
      while (goal.getStatus() != status) {
        goalStatusUpdateCondition.await();
      }
    } catch (InterruptedException e) {
      log.error("Error waiting for goal {} with status {}", goal, status);
    } finally {
      goalStatusUpdateLock.unlock();
    }
  }

  /**
   * Notify EM of goal status changes, so goal can be properly managed (e.g., moved to past goals).
   *
   * @param info GoalInfo of the goal with changed status
   */
  @Override
  public void goalStatusChanged(GoalInfo info) {
    log.debug("[goalStatusChanged] goal: {} status: {}", info.goal, info.status);

    // notify everyone waiting on goal status notifications
    goalStatusUpdateLock.lock();
    try {
      goalStatusUpdateCondition.signalAll();
    } finally {
      goalStatusUpdateLock.unlock();
    }

    // transfer terminated goals to past goals
    if (info.status.isTerminated()) {
      Goal goal = getGoal(info.gid);
      if (goal == null) {
        log.warn("[goalStatusChanged] Could not find goal: {}", info.goal);
        return;
      }

      synchronized (goalsLock) {
        log.trace("[goalStatusChanged] {} have goalsLock", goal);
        transferGoalToPastGoals(goal);
      }
      log.trace("[goalStatusChanged] {} release goalsLock", goal);
    }
  }

  /**
   * Searches for the supplied goal in the pending and active goal collections, moving it to pastGoals and performing
   * the relevant bookkeeping if found
   */
  protected void transferGoalToPastGoals(Goal goal) {
    log.debug("[transferGoalToPastGoals] in method with goal: " + goal);

    synchronized (pendingGoalsLock) {
      log.trace("[transferGoalToPastGoals] {} have pendingGoalsLock", goal);
      PendingGoal pg = removePendingGoal(goal);
      if (pg != null) {
        pastGoals.add(goal);
        pg.notifyOfNoLongerPending();
        log.trace("[transferGoalToPastGoals] {} releasing pendingGoalsLock", goal);
        return;
      }
    }

    synchronized (goalsLock) {
      log.trace("[transferGoalToPastGoals] {} have goalsLock", goal);
      // goal is an active goal
      if (getActiveGoal(goal.getId()) != null) {
        unlockResources(goal);
        removeActiveGoal(goal);
        pastGoals.add(goal);
        log.trace("[transferGoalToPastGoals] {} releasing goalsLock", goal);
        return;
      }
    }

    if (goal.isScheduled()) {
      if (scheduledGoalManager.removeGoal((ScheduledGoal) goal)) {
        pastGoals.add(goal);
      }
    }

    log.debug("[transferGoalToPastGoals] No matching pending or active goal could be found for: {}", goal);
  }

  /**
   * Wait indefinitely for the matching pending goal to have its condition notified. Returns immediately if the goal is
   * not found or pending.
   */
  protected void joinOnPendingGoal(long gid) {
    log.trace("[joinOnPendingGoal] {} in method", gid);
    PendingGoal pg = getPendingGoal(gid);

    // if goal is pending, wait for it to become active (or cancelled)
    if (pg != null) {
      pg.waitForNoLongerPending();
      log.trace("[joinOnPendingGoal] done waiting for pending goal to be updated");
    }
  }

  /**
   * Wait (up to specified milliseconds) for the matching pending goal to have its condition notified. Returns
   * immediately if the goal is not found or pending.
   *
   * @return true: goal not pending or not found. false: goal found and is still pending
   */
  private boolean joinOnPendingGoal(long gid, long millis) {
    log.trace("[joinOnPendingGoal] {}, {} in method", gid, millis);
    PendingGoal pg = getPendingGoal(gid);

    // if goal is pending, wait for it to become active (or cancelled)
    if (pg != null) {
      return pg.waitForNoLongerPending(millis);
    }

    return true; // true == goal not pending
  }

  /**
   * Remove the matching PendingGoal from the pending collection and notify of update
   */
  private PendingGoal removePendingGoal(Goal g) {
    log.debug("[removePendingGoal] {}", g);
    synchronized (pendingGoalsLock) {
      log.trace("[removePendingGoal] {} have pendingGoalsLock", g);
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      int index = 0;
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pendingGoal = pendingGoalIterator.next();
        if (pendingGoal.getGoal().getId() == g.getId()) {
          log.trace("[removePendingGoal] {} found goal", g);
          pendingGoalIterator.remove();
          onPendingGoalUpdated(g, index, UpdateType.REMOVED); //updating signature to remove index removes need for iterator
          return pendingGoal;
        }
        index++;
      }
      log.trace("[removePendingGoal] {} didn't find goal", g);
      return null;
    }
  }

  /**
   * return the index of the supplied PendingGoal in the pending collection (which is ordered by priority). Returns -1
   * if not found.
   */
  private int getIndexOfPendingGoal(PendingGoal pg) {
    log.trace("[getIndexOfPendingGoal] {}", pg.getGoal());
    //Can't do this because this doesn't give any indication of how ties are broken
    //pendingGoals.tailSet(pg, false).size();

    synchronized (pendingGoalsLock) {
      log.trace("[getIndexOfPendingGoal] {} have pendingGoalsLock", pg.getGoal());
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
      log.debug("[cancelGoal] {} canceling pending goal {}", gid, pg.getGoal());
      cancelPendingGoal(pg);
      return true;
    }

    // else if goal to cancel is active
    Goal goal = getActiveGoal(gid);
    if (goal != null) {
      log.debug("[cancelGoal] {} canceling active goal {}", gid, goal);
      return cancelActiveGoal(goal);
    }

    // if goal is scheduled goal
    ScheduledGoal scheduledGoal = scheduledGoalManager.getScheduledGoal(gid);
    if (scheduledGoal != null) {
      log.debug("[cancelGoal] {} canceling scheduled goal {}", gid, goal);
      return scheduledGoalManager.cancelScheduledGoal(scheduledGoal);
    }

    log.warn("[cancelGoal] cannot find current goal for gid {}", gid);
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
    log.debug("[cancelPendingGoal] {}", goal);
    if (goal.getActionInterpreter() == null) {
      goal.setStatus(GoalStatus.CANCELED);
      transferGoalToPastGoals(goal);
    } else {
      //If this goal was previously superseded and has an associated AI, it needs to be canceled
      log.debug("[cancelPendingGoal] {} canceling superseded goal", goal);
      cancelActiveGoal(goal);
    }
  }

  /**
   * Remove a pending goal based on index before it is transferred to active
   */
  public boolean cancelPendingGoalByIndex(int index) {
    log.debug("[cancelPendingGoalByIndex] removing pending goal with index: " + index);
    PendingGoal entry;
    synchronized (pendingGoalsLock) {
      if (index < pendingGoals.size()) {
        Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
        int i = 0;
        while (i < index) {
          pendingGoalsIterator.next();
          i++;
        }
        entry = pendingGoalsIterator.next();
      } else {
        log.warn("[cancelPendingGoalByIndex] Queue is of length: " + pendingGoals.size() + ", cannot remove goal with index: " + index);
        return false;
      }
    }
    cancelPendingGoal(entry);
    return true;
  }

  /**
   * Cancel the supplied active goal and transfer to past goals
   *
   * @param goal
   */
  private boolean cancelActiveGoal(Goal goal) {
    log.debug("[cancelActiveGoal] goal: {}", goal.getPredicate());
    boolean retVal = goal.cancel();

    // if goal doesn't have an AI, remove it from goals list and add to pastGoals
    // if goal does have an AI, the actionComplete method will be called to
    // take care of that
    boolean hasAI = goal.getActionInterpreter() != null;
    if (!hasAI) {
      transferGoalToPastGoals(goal);
    }
    return retVal;
  }

  /**
   * Search all goal collections and suspend the goal with the supplied goal id if found
   *
   * @param gid
   * @return true if the goal was found and had an active status, otherwise false
   */
  public boolean suspendGoal(long gid) {
    Goal g = getGoal(gid);
    log.debug("[suspendGoal] {} {}", gid, g);

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
      log.debug("[suspendGoal] suspending active goal {}", g);
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
    log.trace("[suspendActiveGoal] " + goal.getPredicate());
    if (goal == null) {
      log.warn("[suspendActiveGoal] goal is null.");
    } else if (goal.getStatus() == GoalStatus.SUSPENDED) {
      log.warn("[suspendActiveGoal] goal status is already SUSPENDED");
    } else {
      if (goal.suspend()) {
        onActiveGoalUpdated(goal, GoalStatus.SUSPENDED, UpdateType.UPDATED);
        return true;
      }
    }
    return false;
  }

  public List<Long> suspendSystemGoals() {
    return suspendSystemGoals(rootAgent);
  }

  /**
   * Suspends a subset of an agentTeam's active goals. This method ignores goals with PriorityTier SKIPPENDING and
   * scheduled goals. In order to accommodate some practical agent configurations with input methods' addressees,
   * this method can act on the supplied agent or a single descendant which has an active goal. If multiple team members
   * have disjoint active goals, this method will fail and return null.
   * @param agentTeam
   * @return the suspended goals' ids or null.
   */
  public List<Long> suspendSystemGoals(Symbol agentTeam) {
    log.debug("[suspendSystemGoals] in method");
    List<Long> suspendedIds = new ArrayList<>();
    Set<Goal> activeGoals;
    synchronized (goalsLock) {
      activeGoals = getActiveGoalsHelper(agentTeam, 3, false);
    }

    if (activeGoals == null) {
      return null;
    }

    activeGoals = activeGoals.stream().filter(g -> g.getPriorityTier() != PriorityTier.SKIPPENDING).collect(Collectors.toSet());

    for (Goal g: activeGoals) {
      if (suspendGoal(g.getId())) {
        suspendedIds.add(g.getId());
      }
    }

    return suspendedIds;
  }

  /**
   * Search all goal collections and resume the goal with the supplied goal id if found
   *
   * @param gid
   * @return true if the goal was found and had an active status, otherwise false
   */
  public boolean resumeGoal(long gid) {
    Goal g = getGoal(gid);
    log.debug("[resumeGoal] {} {}", gid, g);

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
      log.debug("[resumeGoal] resuming active goal {}", g);
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
    log.trace("[resumeGoal] " + goal);
    if (goal == null) {
      log.warn("[resumeGoal] goal is null.");
    } else {
      if (goal.resume()) {
        onActiveGoalUpdated(goal, GoalStatus.ACTIVE, UpdateType.UPDATED);
        return true;
      }
    }
    return false;
  }

  public List<Long> resumeSystemGoals() {
    return resumeSystemGoals(rootAgent);
  }

  /**
   * Resumes a subset of suspended goals for the supplied agentTeam and all of its children. This method ignores goals
   * with PriorityTier SKIPPENDING and scheduled goals. In order to accommodate some practical agent configurations with
   * input methods' addressees, this method can act on the supplied agent or a single descendant which has an active
   * goal. If multiple team members have disjoint active goals, this method will fail.
   * @param agentTeam
   * @return the resumed goals' ids
   */
  public List<Long> resumeSystemGoals(Symbol agentTeam) {
    log.debug("[resumeSystemGoals] in method");
    List<Long> resumedIds = new ArrayList<>();
    Set<Goal> activeGoals;
    synchronized (goalsLock) {
      activeGoals = getActiveGoalsHelper(agentTeam, 3, false);
    }

    if (activeGoals == null) {
      return null;
    }

    activeGoals = activeGoals.stream().filter(g -> g.getPriorityTier() != PriorityTier.SKIPPENDING).collect(Collectors.toSet());

    for (Goal g: activeGoals) {
      if (resumeGoal(g.getId())) {
        resumedIds.add(g.getId());
      }
    }

    ////PickAndPlace2024 - Last second addition. Get rid of this, have wrapper script with failure justification and handling if we want this
    //if (resumedIds.isEmpty()) {
    //  Predicate resumedNothingSemantics = Factory.createPredicate("cannotResumeTaskBecauseIHaveNoCurrentTask()");
    //  Utterance utt = new Utterance.Builder()
    //          .setSpeaker(agentTeam)
    //          .setAddressee(Factory.createSymbol("user"))
    //          .setUtteranceType(UtteranceType.REPLY)
    //          .setIsInputUtterance(false)
    //          //.setWords(List.of(statusReport.split(" ")))
    //          .setSemantics(resumedNothingSemantics)
    //          .build();
    //  try {
    //    utt = TRADE.getAvailableService(new TRADEServiceConstraints().name("convertSemanticsToText").argTypes(Utterance.class)).call(Utterance.class, utt);
    //    TRADE.getAvailableService(new TRADEServiceConstraints().name("updateDialogueHistory").argTypes(Utterance.class)).call(void.class, utt);
    //    TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(String.class)).call(void.class, utt.getWordsAsString());
    //  } catch (TRADEException e) {
    //    log.error("[handleConflictingLowerPriorityGoal]", e);
    //  }
    //}


    return resumedIds;
  }

  /**
   * @return A copied list of all current, pending, active, and scheduled goals
   */
  public List<Goal> getCurrentGoals() {
    List<Goal> currentGoals;
    log.trace("[getCurrentGoals] in method");
    synchronized (pendingGoalsLock) {
      log.trace("[getCurrentGoals] have pendingGoalsLock");
      synchronized (goalsLock) {
        log.trace("[getCurrentGoals] have goalsLock");
        currentGoals = getActiveGoals();
        currentGoals.addAll(getPendingGoals());
        currentGoals.addAll(scheduledGoalManager.getScheduledGoals());
      }
    }
    log.trace("[getCurrentGoals] released locks");
    return currentGoals;
  }

  /**
   * Get a copied list of the goals currently undergoing execution.
   *
   * @return list of Goals
   */
  public List<Goal> getActiveGoals() {
    Set<Goal> activeGoals;
    log.trace("[getActiveGoals] in method");
    synchronized (goalsLock) {
      log.trace("[getActiveGoals] have goalsLock");
      activeGoals = getActiveGoalsHelper(rootAgent,1, true);
    }
    log.trace("[getActiveGoals] released goalsLock");
    return new ArrayList<>(activeGoals);
  }

  //TODO: Further divide here by getting persistent goals separately?
  //If an agent can be a member of multiple teams, this implementation is invalid
  /**
   * Get the current goals of the supplied agentTeam
   * @param agent the root agent to gather goals of
   * @param filterLevel One of three options:
   *                    0 - No filtering, includes all goals
   *                    1 - Filters out scheduled goals
   *                    2 - Filters out goals with SKIPPENDING PriorityTier, except for scheduled goals
   *                    3 - Filters out all SKIPPENDING goals, including scheduled ones
   * @param allMembers Add goals of descendents. If true, adds all goals of the supplied agent and all members. If false,
   *                   returns only the goals of the supplied agent or a single member. If multiple members have disjoint
   *                   goals, returns null.
   */
  private Set<Goal> getActiveGoalsHelper(Symbol agent, int filterLevel, boolean allMembers) {
    AgentTeam agentTeam = getAgentTeam(agent);
    Set<Goal> agentTeamGoals = agentTeam.getActiveGoals();
    if (filterLevel == 1) {
      agentTeamGoals = agentTeamGoals.stream().filter(g -> !g.isScheduled()).collect(Collectors.toSet());
    } else if (filterLevel == 2) {
      agentTeamGoals = agentTeamGoals.stream().filter(g -> g.getPriorityTier() != PriorityTier.SKIPPENDING || g.isScheduled()).collect(Collectors.toSet());
    } else if (filterLevel == 3) {
      //scheduled goals should imply skippending priority tier, but adding both explicitly in case some domain is implemented differently.
      agentTeamGoals = agentTeamGoals.stream().filter(g -> g.getPriorityTier() != PriorityTier.SKIPPENDING && !g.isScheduled()).collect(Collectors.toSet());
    }
    Set<Goal> goals = new HashSet<>(agentTeamGoals);

    if (!goals.isEmpty() && !allMembers) {
      return goals;
    }

    boolean foundGoal = false;
    Set<Goal> membersGoals = new HashSet<>();
    for (Symbol member : agentTeam.getMemberNames()) {
      Set<Goal> memberGoals = getActiveGoalsHelper(member, filterLevel, allMembers);
      if (memberGoals == null) {
        return null;
    }
      if (!allMembers) {
        if (!memberGoals.isEmpty()) {
          if (foundGoal) {
            return null;
          } else {
            foundGoal = true;
          }
        }
      }
      membersGoals.addAll(memberGoals);
    }
    goals.addAll(membersGoals);
    return goals;
  }

  /**
   * Get a copied list of the currently managed pending goals waiting to begin execution.
   *
   * @return list of Goals
   */
  public List<Goal> getPendingGoals() {
    return pendingGoals.stream().map(PendingGoal::getGoal).toList();
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
   * Cancel all goals currently found in the pendingGoals collection
   */
  public void cancelAllPendingGoals() {
    log.debug("[cancelAllPendingGoals] in method");
    List<Long> goalIds = new ArrayList<>();
    synchronized (pendingGoalsLock) {
      log.trace("[cancelAllPendingGoals] have pendingGoalsLock");
      //while (!pendingGoals.isEmpty()) {
      //  int size = pendingGoals.size();
      //  cancelPendingGoalByIndex(size - 1);
        //if (pendingGoals.size() == size) {
        //  log.error("[cancelPendingGoals] canceled goal which was not removed from the collection, exiting loop");
        //  return;
        //}
      //}
      for (PendingGoal pg : pendingGoals) {
        goalIds.add(pg.getGoal().getId());
      }
    }
    log.trace("[cancelAllPendingGoals] released pendingGoalsLock, canceling all gather goals");
    for (Long gid : goalIds) {
      log.trace("[cancelAllPendingGoals] canceling {}", gid);
      cancelGoal(gid);
    }
  }

  /**
   * Cancel all goals currently found in either the pending or active Goals collections
   */
  public void cancelAllCurrentGoals() {
    log.debug("[cancelAllCurrentGoals] in method");
    //synchronized (pendingGoalsLock) {
    //  log.trace("[cancelAllCurrentGoals] have pendingGoalsLock");
    //  synchronized (goalsLock) {
    //    log.trace("[cancelAllCurrentGoals] have goalsLock");
        scheduledGoalManager.cancelAllScheduledGoals();
        cancelAllPendingGoals();
        cancelAllActiveGoals();
    //  }
    //}
    //log.trace("[cancelAllCurrentGoals] released locks");
  }

  /**
   * Cancel all goals currently found in the activeGoals colection
   */
  public void cancelAllActiveGoals() {
    Set<Goal> goalsToCancel = new HashSet<>();
    synchronized (goalsLock) {
      for (Symbol agentTeam : agentTeams.keySet()) {
        for (Goal g : agentTeams.get(agentTeam).getActiveGoals()) {
          if (!g.getPredicate().getName().equals("listen")) {
            goalsToCancel.add(g);
          }
        }
      }
    }
    for (Goal g : goalsToCancel) {
      cancelActiveGoal(g);
    }
  }

  public List<Long> cancelSystemGoals() {
    return cancelSystemGoals(rootAgent);
  }

  /**
   * Cancels a subset of active goals for the supplied agentTeam and all of its children. This method ignores goals with
   * PriorityTier SKIPPENDING and scheduled goals. In order to accommodate some practical agent configurations with
   * input methods' addressees, this method can act on the supplied agent or a single descendant which has an active
   * goal. If multiple team members have disjoint active goals, this method will fail.
   * @param agentTeam
   * @return the canceled goals' ids
   */
  public List<Long> cancelSystemGoals(Symbol agentTeam) {
    log.debug("[cancelSystemGoals] in method");
    List<Long> canceledIds = new ArrayList<>();
    Set<Goal> activeGoals;
    synchronized (goalsLock) {
      activeGoals = getActiveGoalsHelper(agentTeam, 3, false);
    }

    if (activeGoals == null) {
      return null;
    }

    activeGoals = activeGoals.stream().filter(g -> g.getPriorityTier() != PriorityTier.SKIPPENDING).collect(Collectors.toSet());

    for (Goal g: activeGoals) {
      if (cancelGoal(g.getId())) {
        canceledIds.add(g.getId());
      }
    }

    return canceledIds;
  }

  /**
   * Get a list of the corresponding predicate representations of all goals currently found in the pending collection
   */
  public List<Predicate> getPendingGoalsPredicates() {
    log.trace("[getPendingGoalsPredicates] in method");
    List<Predicate> goalPreds = new ArrayList<>();
    synchronized (pendingGoalsLock) {
      log.trace("[getPendingGoalsPredicates] have pendingGoalsLock");
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pendingGoal = pendingGoalIterator.next();
        goalPreds.add(pendingGoal.getGoal().getPredicate());
      }
    }
    log.trace("[getPendingGoalsPredicates] released pendingGoalsLock");
    return goalPreds;
  }

  /**
   * Get a list of the corresponding predicate representations of all goals currently found in the active collection.
   * If wrap == true -> result is wrapped in a predicate with the goal's status as its name
   */
  public List<Predicate> getSystemGoalsPredicates(boolean wrap) {
    return getSystemGoalsPredicates(rootAgent, wrap);
  }

  /**
   * Get a list of the corresponding predicate representations of all goals currently found in the active collection
   * for the supplied actor. If wrap == true -> result is wrapped in a predicate with the goal's status as its name
   */
  public List<Predicate> getSystemGoalsPredicates(Symbol actor, boolean wrap) {
    log.trace("[getSystemGoalsPredicates] {}", actor);
    return getSystemGoalsPredicatesHelper(actor.toUntyped(), new ArrayList<>(), wrap);
  }

  private List<Predicate> getSystemGoalsPredicatesHelper(Symbol actor, List<Predicate> goalPreds, boolean wrap) {
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      return goalPreds;
    }

    for (Goal g : agentTeam.getActiveGoals()) {
      if (g.getPriorityTier() != PriorityTier.SKIPPENDING) {
        if (wrap) {
          goalPreds.add(Factory.createPredicate(g.getStatus().toString().toLowerCase(), g.getPredicate()));
        } else {
          goalPreds.add(g.getPredicate());
        }
      }
    }

    for (Symbol member : agentTeam.getMemberNames()) {
      goalPreds.addAll(getSystemGoalsPredicates(member,wrap));
    }

    return goalPreds;
  }

  /**
   * Get the predicate corresponding to the goal with the highest priority found in the pending collection
   */
  public Predicate getNextGoalPredicate() {
    log.trace("[getNextGoalPredicate] in method");
    synchronized (pendingGoalsLock) {
      log.trace("[getNextGoalPredicate] have pendingGoalsLock");
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      if (pendingGoalIterator.hasNext()) {
        log.trace("[getNextGoalPredicate] releasing pendingGoalsLock");
        return pendingGoalIterator.next().getGoal().getPredicate();
      } else {
        log.trace("[getNextGoalPredicate] releasing pendingGoalsLock");
        return Factory.createPredicate("none()");
      }
    }
  }

  /**
   * Get the predicate corresponding to the goal with the highest priority for the supplied agent found in the
   * pending collection
   */
  public Predicate getNextGoalPredicate(Symbol agent) {
    log.trace("[getNextGoalPredicate] {}", agent);
    agent = agent.toUntyped();
    synchronized (pendingGoalsLock) {
      log.trace("[getNextGoalPredicate] {} have pendingGoalsLock", agent);
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pg = pendingGoalIterator.next();
        if (agent.equals(pg.getGoal().getActor().toUntyped())) {
          log.trace("[getNextGoalPredicate] {} releasing pendingGoalsLock", agent);
          return pg.getGoal().getPredicate();
        }
      }
    }
    log.trace("[getNextGoalPredicate] {} releasing pendingGoalsLock", agent);
    return Factory.createPredicate("none()");
  }

  /**
   * Get the goal for a particular goal ID. This checks pending, current, and past goals.
   *
   * @param gid goal id.
   * @return the goal corresponding to the ID.
   */
  public Goal getGoal(long gid) {
    log.trace("[getGoal] {}", gid);
    Goal g = null;
    PendingGoal pg = getPendingGoal(gid);
    if (pg != null) {
      log.trace("[getGoal] {} is pending goal", gid);
      g = pg.getGoal();
    }
    if (g == null) {
      log.trace("[getGoal] {} is active goal", gid);
      g = getActiveGoal(gid);
    }
    if (g == null) {
      log.trace("[getGoal] {} is past goal", gid);
      g = getPastGoal(gid);
    }
    if (g == null) {
      log.trace("[getGoal] {} is scheduled goal", gid);
      g = scheduledGoalManager.getScheduledGoal(gid);
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
    log.trace("[getCurrentGoal] {}, {}", actor, index);
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
    log.trace("[getActiveGoal] {}, {}", actor, index);
    return getGoalHelper(getActiveGoals(), actor, index);
  }

  /**
   * Get an active goal
   *
   * @param gid goal id
   * @return the goal
   */
  public Goal getActiveGoal(long gid) {
    log.trace("[getActiveGoal] {}", gid);
    synchronized (goalsLock) {
      log.trace("[getActiveGoal] {} have goalsLock", gid);
      Goal g;
      for (Symbol agent : agentTeams.keySet()) {
        g = agentTeams.get(agent).getActiveGoal(gid);
        if (g != null) {
          log.trace("[getActiveGoal] {} releasing goalsLock", gid);
          return g;
        }
      }
    }
    log.trace("[getActiveGoal] {} released goalsLock", gid);
    return null;
  }

  /**
   * Get a pending goal
   *
   * @param gid goal id
   * @return the goal
   */
  public PendingGoal getPendingGoal(long gid) {
    log.trace("[getPendingGoal] {} ", gid);
    synchronized (pendingGoalsLock) {
      log.trace("[getPendingGoal] {} have pendingGoalsLock", gid);
      for (PendingGoal pg : pendingGoals) {
        if (pg.getGoal().getId() == gid) {
          log.trace("[getPendingGoal] {} releasing pendingGoalsLock", gid);
          return pg;
        }
      }
    }
    log.trace("[getPendingGoal] {} released pendingGoalsLock", gid);
    return null;
  }

  /**
   * Get a copied list of all scheduled goals which have either yet to be submitted, or are currently executing.
   *
   * @return shallow copy of scheduled goals
   */
  public List<ScheduledGoal> getScheduledGoals() {
    return scheduledGoalManager.getScheduledGoals();
  }

  /**
   * Get ScheduledGoal instance with goal ID.
   *
   * @param goalId scheduled goal ID
   * @return ScheduledGoal or null
   */
  public ScheduledGoal getScheduledGoal(long goalId) {
    return scheduledGoalManager.getScheduledGoal(goalId);
  }

  /**
   * Get the goal for a particular past goal ID.
   *
   * @param gid goal id.
   * @return the goal corresponding to the ID.
   */
  public Goal getPastGoal(long gid) {
    log.trace("[getPastGoal] {} ", gid);
    synchronized (pastGoals) {
      log.trace("[getPastGoal] {} have pastGoals lock", gid);
      for (Goal g : pastGoals) {
        if (g.getId() == gid) {
          log.trace("[getPastGoal] {} releasing pastGoals lock", gid);
          return g;
        }
      }
    }
    log.debug("[getPastGoal] {} no past goal found", gid);
    return null;
  }

  /**
   * Get a list of the currently running goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  protected List<Goal> getActiveGoals(Predicate queryGoal) {
    log.debug("[getActiveGoals] {}", queryGoal);
    return getGoalHelper(getActiveGoals(), queryGoal);
  }

  /**
   * Get a list of all past goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  public List<Goal> getPastGoals(Predicate queryGoal) {
    log.debug("[getPastGoals] {}", queryGoal);
    return getGoalHelper(getPastGoals(), queryGoal);
  }

  /**
   * Get a list of all current pending and/or active goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  public List<Goal> getCurrentGoals(Predicate queryGoal) {
    log.debug("[getCurrentGoals] {}", queryGoal);
    return getGoalHelper(getCurrentGoals(), queryGoal);
  }

  /**
   * Get a list of all pending, active, and past goals matching the goalPredicate query.
   *
   * @return goals with matching goalPredicate, or null if no matching goal found
   */
  public List<Goal> getAllGoals(Predicate queryGoal) {
    log.debug("[getAllGoals] {}", queryGoal);
    synchronized (pendingGoalsLock) {
      log.trace("[getAllGoals] {} have pendingGoalsLock", queryGoal);
      synchronized (pastGoals) {
        log.trace("[getAllGoals] {} have pastGoalsLock", queryGoal);
        synchronized (goalsLock) {
          log.trace("[getAllGoals] {} have goalsLock", queryGoal);
          List<Goal> goals = getCurrentGoals(queryGoal);
          goals.addAll(getPastGoals(queryGoal));
          log.trace("[getCurrentGoals] {} releasing all locks", queryGoal);
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
    log.trace("[getGoalStatus] {}", gid);
    Goal goal = getGoal(gid);

    if (goal != null) {
      return goal.getStatus();
    } else {
      return GoalStatus.UNKNOWN;
    }
  }

  /**
   * Query the priority tier of a particular goal.
   *
   * @param gid the ID of the goal to check on
   * @return PriorityTier object indicating the priority tier
   */
  public PriorityTier getGoalPriorityTier(long gid) {
    log.trace("[getGoalPriorityTier] {}", gid);
    Goal goal = getGoal(gid);

    if (goal != null) {
      return goal.getPriorityTier();
    } else {
      return PriorityTier.UNINITIALIZED;
    }
  }

  /**
   * Query the priority of a particular goal.
   *
   * @param gid the ID of the goal to check on
   * @return long indicating the priority value
   */
  public long getGoalPriority(long gid) {
    log.trace("[getGoalPriorityTier] {}", gid);
    Goal goal = getGoal(gid);

    if (goal != null) {
      return goal.getPriority();
    } else {
      return -1;
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
    log.trace("[getActionStatus] {}", gid);
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
    log.trace("[getGoalFailConditions] {}", gid);

    Goal goal = getGoal(gid);

    if (goal != null) {
      return goal.getFailConditions();
    } else {
      return new ConditionJustification(true, Factory.createPredicate("reason(unknown)"));
    }
  }

  /**
   * Modify existing scheduled goal based on update predicate.
   *
   * @param goalId ID of scheduled goal
   * @param update update predicate
   * @return if update was successful
   */
  public boolean updateScheduledGoal(long goalId, Predicate update) {
    return scheduledGoalManager.updateScheduledGoal(goalId, update);
  }

  private Goal getGoalHelper(List<Goal> currGoals, Symbol actor, int index) {
    // filter goals for actor (if not null)
    if (actor != null) {
      currGoals = currGoals.stream().filter(goal -> (goal.getActor() == null || goal.getActor().equals(actor))).collect(Collectors.toList());
    }

    // sort using goal start time
//        currGoals.sort((a, b) -> Long.compare(b.getStartTime(), a.getStartTime()));
    currGoals.sort((a, b) -> Long.compare(b.getId(), a.getId()));
    log.trace("[getGoalHelper(" + actor + "," + index + ")] time ordered goals: " + Arrays.toString(currGoals.toArray()));

    //
    if (index < 0) {
      index = currGoals.size() + index;
    }

    Goal returnGoal = null;
    if (index < currGoals.size()) {
      returnGoal = currGoals.get(index);
    } else {
      log.trace("[getGoalHelper(" + actor + "," + index + ")] no matching goal found.");
    }
    return returnGoal;
  }

  private List<Goal> getGoalHelper(List<Goal> goalCollection, Predicate queryGoal) {
    log.trace("[getGoalHelper] {}. {}", goalCollection, queryGoal);
    List<Goal> matchingGoals = new ArrayList<>();
    for (Goal currGoal : goalCollection) {
      if (currGoal.getPredicate().instanceOf(queryGoal)) {
        matchingGoals.add(currGoal);
      }
    }
    return matchingGoals;
  }

  ////////// Action Learning //////////
  @Action
  @TRADEService
  public void waitForActionLearningStart(Symbol actor, Predicate newAction) {
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[waitForActionLearningStart] unknown agent supplied {}", actor);
    } else {
      agentTeam.waitForActionLearningStart(newAction);
    }
  }

  @OnInterrupt(
          onCancelServiceCall = "cancelActionLearning(?actor, ?newAction)",
          onSuspendServiceCall = "pauseActionLearning(?actor, ?newAction)",
          onResumeServiceCall = "resumeActionLearning(?actor, ?newAction)"
  )
  @Action
  @TRADEService
  public boolean learnAction(Symbol actor, Predicate newAction) {
    log.debug("[learnAction] {}, {}", actor, newAction);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[learnAction] unknown actor supplied {}", actor);
      return false;
    }

    Set<Symbol> relevantAgents = getRelevantAgents(actor);
    for (Symbol relevantAgent : relevantAgents) {
      AgentTeam relevantAgentTeam = getAgentTeam(relevantAgent);
      if (!relevantAgent.getName().equals(relevantAgentTeam.getName().getName()) && relevantAgentTeam.getLearningStatus() == ActionLearningStatus.ACTIVE) {
        log.error("[learnAction] Trying to start learning for an actor which already has an ancestor or child actively learning. New: {} Current: {}", actor, relevantAgent);
        return false;
      }
    }

    return agentTeam.learnAction(newAction);
  }

  @Action
  @TRADEService
  public boolean resumeActionLearning(Symbol actor, Predicate newAction) {
    log.debug("[resumeActionLearning] {}, {}", actor, newAction);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[resumeActionLearning] unknown actor supplied {}", actor);
      return false;
    }

    Set<Symbol> relevantAgents = getRelevantAgents(actor);
    for (Symbol relevantAgent : relevantAgents) {
      AgentTeam relevantAgentTeam = getAgentTeam(relevantAgent);
      if (relevantAgentTeam.getLearningStatus() == ActionLearningStatus.ACTIVE) {
        log.error("[resumeActionLearning] Trying to resume learning for an actor which already has an ancestor or child actively learning. New: {} Current: {}", actor, relevantAgent);
        return false;
      }
    }

    return agentTeam.resumeActionLearning(newAction);
  }

  @Action
  @TRADEService
  public boolean endActionLearning(Symbol actor, Predicate newAction) {
    log.debug("[endActionLearning] {}, {}", actor, newAction);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[endActionLearning] unknown actor supplied {}", actor);
      return false;
    }
    return agentTeam.endActionLearning(newAction);
  }

  @Action
  @TRADEService
  public boolean pauseActionLearning(Symbol actor, Predicate newAction) {
    log.debug("[pauseActionLearning] {}, {}", actor, newAction);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[pauseActionLearning] unknown actor supplied {}", actor);
      return false;
    }
    return agentTeam.pauseActionLearning(newAction);
  }

  @Action
  @TRADEService
  public boolean cancelActionLearning(Symbol actor, Predicate newAction) {
    log.debug("[cancelActionLearning] {}, {}", actor, newAction);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[cancelActionLearning] unknown actor supplied {}", actor);
      return false;
    }
    return agentTeam.cancelActionLearning(newAction);
  }

  @Action
  @TRADEService
  public boolean resumeActionLearning(Symbol actor) {
    log.debug("[resumeActionLearning] {}", actor);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[resumeActionLearning] unknown actor supplied {}", actor);
      return false;
    }

    Predicate activeSignature = agentTeam.getActiveLearningSignature();
    if (activeSignature == null) {
      log.error("[resumeActionLearning] {} does not have an active learning state", actor);
      return false;
    }

    return resumeActionLearning(actor,activeSignature);
  }

  @Action
  @TRADEService
  public boolean endActionLearning(Symbol actor) {
    log.debug("[endActionLearningPrimitive] {}, {}", actor);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[endActionLearningPrimitive] unknown actor supplied {}", actor);
      return false;
    }

    Predicate activeSignature = agentTeam.getActiveLearningSignature();
    if (activeSignature == null) {
      log.error("[endActionLearningPrimitive] {} does not have an active learning state", actor);
      return false;
    }

    return endActionLearning(actor,activeSignature);
  }

  @Action
  @TRADEService
  public boolean pauseActionLearning(Symbol actor) {
    log.debug("[pauseActionLearning] {}, {}", actor);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[pauseActionLearning] unknown actor supplied {}", actor);
      return false;
    }
    Predicate activeSignature = agentTeam.getActiveLearningSignature();
    if (activeSignature == null) {
      log.error("[pauseActionLearning] {} does not have an active learning state", actor);
      return false;
    }

    return pauseActionLearning(actor,activeSignature);
  }

  @Action
  @TRADEService
  public boolean cancelActionLearning(Symbol actor) {
    log.debug("[cancelActionLearning] {}, {}", actor);
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[cancelActionLearning] unknown actor supplied {}", actor);
      return false;
    }
    Predicate activeSignature = agentTeam.getActiveLearningSignature();
    if (activeSignature == null) {
      log.error("[cancelActionLearning] {} does not have an active learning state", actor);
      return false;
    }

    return cancelActionLearning(actor,activeSignature);
  }

  @TRADEService
  @Action
  public ActionLearningStatus getLearningStatus(Symbol actor) {
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[getLearningStatus] unknown actor supplied {}", actor);
      return ActionLearningStatus.NONE;
    }
    return agentTeam.getLearningStatus();
  }

  @TRADEService
  @Action
  public void changeLearningExecution(Symbol actor, Symbol status) {
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[changeLearningExecution] unknown actor supplied {}", actor);
    } else {
      agentTeam.changeLearningExecution(status);
    }
  }

  @Action
  @TRADEService
  public void modifyAction(Symbol actor, Predicate action, Predicate modification, Predicate location) {
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.error("[modifyAction] unknown actor supplied {}", actor);
    } else {
      agentTeam.modifyAction(action, modification, location);
    }
  }

  //TODO: Revisit location and interaction of persistent learned action functionality with rest of DB/EM.
  //      Weird because file doesn't live in resources
  //This function assumes it is being used alongside persistent taught actions written to file. Use forgetHow otherwise
  @TRADEService
  public Justification forgetTaughtAction(Predicate actionSignature) {
    try {
      List<ActionDBEntry> actions = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActionsBySignature").argTypes(Predicate.class)).call(List.class, actionSignature);
      if (actions.isEmpty()) {
        return new ConditionJustification(false, Factory.createPredicate("knowAction", actionSignature));
      }
      actions = actions.stream().filter(action -> action.getDBFile() == null || action.getDBFile().equals(learnedActionsFile)).toList();
      if (actions.isEmpty()) {
        return new ConditionJustification(false, Factory.createPredicate("taughtAction", actionSignature));
      }
      for (ActionDBEntry action : actions) {
        //Remove from DB
        Database.getInstance().removeActionDBEntry(action);
        //Remove from file
        String actionFile = action.getDBFile();
        if (actionFile == null) {
          actionFile = learnedActionsFile;
        }
        if (actionFile != null) {
          List<ActionDBEntry> learnedActions = loadLearnedActions(false);
          boolean removed = learnedActions.remove(action);
          if (!removed) {
            //TODO: Look into - get two different ActionDBEntry objects when retrieving from DB vs from file if learned
            //  during this instance of TRACS. If present on startup, then the same object is returned and matches above
            removed = learnedActions.removeIf(a -> a.getType().equals(action.getType()) && a.getRoles().size() == action.getRoles().size());
            if (!removed) {
              log.warn("action was not found in learned file");
            }
          }
          ActionScriptLanguageWriter writer = new ActionScriptLanguageWriter();
          writer.writeToFile(learnedActions, learnedActionsFile);
        }
      }

      //Ava learned presentation/media management hack
      if (actionSignature.getName().length() > 7 && actionSignature.getName().startsWith("present")) {
        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("deletePresentationSlides").argTypes(String.class)).call(void.class, actionSignature.getName().split("present")[1]);
        } catch (TRADEException ignored){}
      }

      return new ConditionJustification(true);
    } catch (TRADEException e) {
      log.error("could not find service", e);
      return new ConditionJustification(false,Factory.createPredicate("known","error"));
    }
  }
  //forgetAllTaughtActions - Too drastic to expose?

  public List<ActionDBEntry> loadLearnedActions(boolean addToDB) {
    if (learnedActionsFile != null) {
      try {
        String s = Files.readString(Path.of(learnedActionsFile));
        if (!s.isEmpty()) {
          ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
          List<ActionDBEntry> entries = parser.parseFromString(learnedActionsFile, new ArrayList<>(), s);
          if (addToDB) {
            for (ActionDBEntry entry : entries) {
              Database.getInstance().addActionDBEntry(entry);
            }
          }
          return entries;
        }
      } catch (IOException e) {
        log.info("[initializeDB] Supplied learnedActionsFile does not yet exist");
      }
    }
    return new ArrayList<>();
  }

  @TRADEService
  public List<Predicate> getLearnedActionPredicates() {
    List<ActionDBEntry> learnedActions = loadLearnedActions(false);
    return new ArrayList<>(learnedActions.stream().map(dbEntry -> dbEntry.getSignature(false)).toList());
  }

  @TRADEService
  public boolean isLearnedAction(Predicate actionSignature) {
    try {
      List<ActionDBEntry> actions = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActionsBySignature").argTypes(Predicate.class)).call(List.class, actionSignature);
      actions = actions.stream().filter(action -> action.getDBFile() == null || action.getDBFile().equals(learnedActionsFile)).toList();
      return !actions.isEmpty();
    } catch (TRADEException e) {
      log.error("[isLearnedAction] error getting action list from signature");
      return false;
    }
  }

  @TRADEService
  public boolean waitForLearningStateUpdate(Symbol agent, Predicate actionSignature) {
    AgentTeam agentTeam = getAgentTeam(agent);
    if (agentTeam != null) {
      return agentTeam.waitForLearningStateUpdate(actionSignature);
    }
    return false;
  }

  @TRADEService
  public void interruptWaitForLearningStateUpdate(Symbol agent, Predicate actionSignature) {
    AgentTeam agentTeam = getAgentTeam(agent);
    if (agentTeam != null) {
      agentTeam.interruptWaitForLearningStateUpdate(actionSignature);
    }
  }

  /**
   * Returns true if the supplied goal should be intercepted by Action Learning.
   * Checks the actor of the supplied goal and any of its ancestors for matching criteria
   */
  private boolean handOffToLearning(Goal g) {
    AgentTeam originalAgentTeam = getAgentTeam(g.getActor());
    //TODO: Is this correct that SKIPPENDING goals globally shouldn't be allowed to be taught as action steps?
    if (originalAgentTeam.shouldIgnore(g) || g.getPriorityTier().equals(PriorityTier.SKIPPENDING)) {
      return false;
    }

    AgentTeam agentTeam = originalAgentTeam;
    while (agentTeam != null) {
      if (agentTeam.getLearningStatus() == ActionLearningStatus.ACTIVE) {
        //What should happen here?
        if (g.getPriorityTier().equals(PriorityTier.URGENT)) {
          log.warn("[handOffToLearning] encountered goal with URGENT priority, were you trying to supersede action learning?");
        }
        log.debug("[handOffToLearning] handing {} off to action learning", g);
        agentTeam.addLearningGoal(g);
        if (!agentTeam.shouldExecute()) {
          pastGoals.add(g);
        }
        return true;
      }
      agentTeam = agentTeam.getParentTeam();
    }

    return false;
  }

  ////////// Freeze Functionality  //////////

  /**
   * 'Freezes' a team or agent and blocks until unfrozen. A frozen agent will
   * have all current goals suspended and will not be able to begin execution of
   * any new ones. Similarly, all members of a frozen team will be frozen.
   */
  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "endFreeze(?agent)", onSuspendServiceCall = "endFreeze(?agent)")
  public void freeze(Symbol agent) {
    log.debug("[freeze] {}", agent);
    agent = agent.toUntyped();
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
      log.trace("[freeze] {} waiting on condition ...", agent);
      freezeLockCondition.await();
    } catch (InterruptedException e) {
      log.error("[freeze]", e);
    } finally {
      freezeLock.unlock();
    }
    log.trace("[freeze] {}, done waiting on condition", agent);

    if (!alreadyFrozen) {
      freezeLocks.remove(agent);
      freezeConditions.remove(agent);
    }
  }

  /**
   * Ends the frozen status for the supplied agent/team
   */
  @Action
  @TRADEService
  public void endFreeze(Symbol agent) {
    log.debug("[endFreeze] {}", agent);
    agent = agent.toUntyped();
    //Unfreeze the AgentTeam and all children
    AgentTeam agentTeam = getAgentTeam(agent);
    if (agentTeam == null) {
      log.warn("[endFreeze] supplied agent is not found in the hierarchy: {}", agent);
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

    log.trace("[endFreeze] {}, getting freeze lock ...", agent);
    freezeLock.lock();
    try {
      log.trace("[endFreeze] {}, obtained lock, signalling condition", agent);
      freezeCondition.signalAll();
    } finally {
      freezeLock.unlock();
    }
    log.trace("[endFreeze] {}, signalled condition", agent);
  }

  //TODO: implement and make configurable similar to skipsQueue
  //      Where should this live?

  /**
   * Call ignoreTentativeAccept if service is present in the system
   */
  @TRADEService
  @Action
  public boolean checkIgnoreTentativeAccept(Predicate goalPred) {
    Collection<TRADEServiceInfo> tsis = TRADE.getAvailableServices(new TRADEServiceConstraints().name("ignoreTentativeAccept").argTypes(Predicate.class));

    for (TRADEServiceInfo tsi : tsis) {
      try {
        if (tsi.call(Boolean.class, goalPred)) {
          return true;
        }
      } catch (TRADEException e) {
        log.error("[checkIgnoreTentativeAccept] error calling ignoreTentativeAccept", e);
      }
    }
    return false;
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

  protected AgentTeam getAgentTeam(Symbol name) {
    name = name.toUntyped();
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
   * Get main state machine.
   * NOTE: the @TRADEService should be removed once there's a cleaner way to get a plan for a goal
   *       without submitting a goal for execution. This is used in asl in a few places.
   *
   * @return
   */
  @TRADEService
  public StateMachine getStateMachine() {
    return sm;
  }

  public RootContext getRootContext() {
    return rootContext;
  }

  public void setActionLearningGuiFlag(boolean flag) {
    displayActionLearningGui = flag;
  }

  public void setScheduledGoalsFile(String filename) {
    scheduledGoalManager.setScheduledGoalsFile(filename);
  }

  public void setLearnedActionsFile(String filename) {
    learnedActionsFile = filename;
  }

  public String getLearnedActionsFile() {
    return learnedActionsFile;
  }

  public boolean getActionLearningGuiFlag() {
    return displayActionLearningGui;
  }

  public void writeScheduledGoalsToFile() {
    scheduledGoalManager.writeScheduledGoalsToFile();
  }
  /**
   * Shutdown this ExecutionManager. All currently managed goals are canceled, ending the ActionInterpreter.
   * Note: the goals are automagically moved to the pastGoals list by the ActionListener mechanism.
   */
  public void shutdown() {
    log.info("Shutting down and cancelling all goals...");

    // cancel all scheduled and pending and active goals and wait for them to terminate
    writeScheduledGoalsToFile();
    getScheduledGoals().forEach(goal -> cancelGoal(goal.getId()));
    getPendingGoals().forEach(goal -> cancelGoal(goal.getId()));
    List<Goal> activeGoals = getActiveGoals();
    activeGoals.forEach(goal -> cancelGoal(goal.getId()));
    activeGoals.forEach(goal -> joinOnGoal(goal.getId()));

    scheduledGoalManager.shutdown();
    memoryManager.shutdown();
    sm.shutdown();
    executor.shutdown();
  }

  /**
   * Entry point to submit a context tree to be executed
   *
   * TODO: Should this be implemented by each subclass? - probably
   *       this is only used in PA right now and explicitly for SIMULATE_PERFORMANCE exec types
   *
   * @param g       the goal
   * @param context the context tree which will be executed
   * @return the context which will be executed first
   */
  public Context submitGoalContext(Goal g, Context context) {
    // register "this" as a goal listener to be notified when goal status changes
    g.addListener(this);

    ActionInterpreter ai = ActionInterpreter.createInterpreterFromExecutionTree(g, context);
    Context simulationStartStep = ai.getCurrentStep();
    log.debug("Starting step " + simulationStartStep.getSignatureInPredicateForm());
    for (ActionListener al : aiListeners) {
      ai.addListener(al);
    }
    startActionInterpreter(ai);
    return simulationStartStep;
  }
}
