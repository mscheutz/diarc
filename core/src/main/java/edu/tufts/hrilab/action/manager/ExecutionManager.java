/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
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
import edu.tufts.hrilab.action.notification.GoalManagerNotifier;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.ToLongFunction;
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
    return compareGoal(g1.getPriorityTier(), effectivePriority(g1), g1.getId(),
            g2.getPriorityTier(), effectivePriority(g2), g2.getId());
  };

  private static final Logger log = LoggerFactory.getLogger(ExecutionManager.class);
  private final ManagerCoordinator coordinator = new ManagerCoordinator();
  private final SchedulingPolicy schedulingPolicy;

  private static final class StartingGoal {
    final CompletableFuture<ActionStatus> completion = new CompletableFuture<>();
    final CompletableFuture<Void> startupFinished = new CompletableFuture<>();
    boolean cancelRequested;
    boolean dispatched;
  }

  private final Map<Goal, StartingGoal> startingGoals = new HashMap<>();
  private final Map<Goal, StartingGoal> executionAttempts = new HashMap<>();
  private final Map<Goal, CompletableFuture<Void>> submissionsInFlight = new HashMap<>();
  private final Map<Goal, GoalStatus> preemptions = new HashMap<>();
  private final Set<Goal> cancelingGoals = new HashSet<>();
  private final Set<Goal> resumingGoals = new HashSet<>();
  private volatile boolean shuttingDown;
  private final ScheduledThreadPoolExecutor policyTimer = new ScheduledThreadPoolExecutor(1, r -> {
    Thread thread = new Thread(r, "goal-manager-policy-timer");
    thread.setDaemon(true);
    return thread;
  });
  private final Map<Goal, ScheduledFuture<?>> deadlineTimers = new HashMap<>();
  private final Map<Goal, Long> pendingSince = new HashMap<>();
  private final Map<Goal, Long> agedPriorities = new ConcurrentHashMap<>();
  private long agingIntervalMillis;
  private ScheduledFuture<?> agingTimer;
  private final ConcurrentMap<Long, CompletableFuture<Void>> scheduledContinuations = new ConcurrentHashMap<>();
  private final Map<Long, Goal> continuationGoals = new HashMap<>();
  private final ConcurrentMap<Thread, AtomicInteger> operationThreads = new ConcurrentHashMap<>();
  private final Object operationMonitor = new Object();
  private int maxManagedGoals;
  private volatile int maxPastGoals;
  private final Object shutdownLock = new Object();
  private CompletableFuture<Void> shutdownCompletion;
  private volatile boolean shutdownEscalated;
  private volatile String shutdownPhase = "RUNNING";
  private final String recoverySessionId = UUID.randomUUID().toString();
  private volatile GoalRecoveryStore recoveryStore;
  private volatile String recoveryFailure;
  private final List<GoalRecoveryRecord> recoveryRecords = new ArrayList<>();

  public ExecutionManager() {
    this(SchedulingPolicy.CONCURRENT);
  }

  ExecutionManager(SchedulingPolicy schedulingPolicy) {
    this.schedulingPolicy = Objects.requireNonNull(schedulingPolicy);
    policyTimer.setRemoveOnCancelPolicy(true);
  }

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

  // When nested, acquire pendingGoalsLock, resourceLock, then goalsLock.
  protected final Object resourceLock = new Object();

  private final Map<Goal, ResourceClaims> resourceAllocations = new HashMap<>();

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
  private final ConcurrentMap<Long, Goal> registeredGoals = new ConcurrentHashMap<>();
  private final ConcurrentMap<Long, Goal> learningGoals = new ConcurrentHashMap<>();

  /**
   * Handles thread(s) that execute ActionInterpreters.
   */
  private final ExecutorService executor = new GoalManagerExecutor();

  /**
   * Handles currently scheduled goals.
   */
  private ScheduledGoalManager scheduledGoalManager;
  private GoalManagerNotifier notifier;
  private Function<Predicate, PriorityTier> priorityTierForGoal = goal -> PriorityTier.NORMAL;
  private ToLongFunction<Predicate> priorityForGoal = goal -> 1L;

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
  private final List<ActionListener> aiListeners = new CopyOnWriteArrayList<>();

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

  private static final class GoalWaitState {
    long interruptVersion;
    int waiters;
    boolean pendingInterrupt;
  }

  private final Map<Long, GoalWaitState> goalWaitStates = new HashMap<>();

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
    ResourceClaims claims = getResourceClaims(g);
    synchronized (pendingGoalsLock) {
      log.trace("[resourceConflictInPending] have pendingGoalsLock");
      Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
      int i = 0;
      while (pendingGoalsIterator.hasNext() && i < maxIndex) {
        PendingGoal pg = pendingGoalsIterator.next();
        i++;
        if (claims.conflictsWith(getResourceClaims(pg.getGoal()))) {
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
    if (!coordinator.isOwner()) {
      coordinator.run(this::activateValidPendingGoals);
      return;
    }
    refreshPendingOrder();
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
    if (!coordinator.isOwner()) {
      return coordinator.call(() -> activateNextValidPendingGoal(startSearchIndex));
    }
    if (shuttingDown) {
      return -1;
    }
    log.trace("[activateNextPendingGoal] in method");
    synchronized (pendingGoalsLock) {
      log.trace("[activateNextPendingGoal] have pendingGoals lock");
      synchronized (resourceLock) {
        log.trace("[activateNextPendingGoal] have resourceLock");
        //iterate through pending goals in order of priority
        Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
        int goalIndex = 0;
        while (pendingGoalsIterator.hasNext()) {
          PendingGoal pg = pendingGoalsIterator.next();
          if (pg.getGoal().getStatus().isTerminated() || cancelingGoals.contains(pg.getGoal())) {
            goalIndex++;
            continue;
          }
          if (goalIndex >= startSearchIndex) {
            if (!resourceConflictInPending(pg.getGoal(), goalIndex)
                    && lockResources(pg.getGoal())) {
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
    return !getResourceClaims(g).used.isEmpty();
  }

  private ResourceClaims getResourceClaims(Goal goal) {
    return new ResourceClaims(getRequiredResourcesForGoal(goal), getHeldResourcesForGoal(goal));
  }

  protected Set<Goal> getResourceConflictingActiveGoals(Goal goal) {
    ResourceClaims claims = getResourceClaims(goal);
    synchronized (resourceLock) {
      Set<Goal> conflicts = getResourceConflictingActiveGoals(claims.used);
      resourceAllocations.forEach((active, allocation) -> {
        if (claims.conflictsWith(allocation)) {
          conflicts.add(active);
        }
      });
      conflicts.remove(goal);
      return conflicts;
    }
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

      if (schedulingPolicy == SchedulingPolicy.SINGLE_GOAL) {
        resourceSet.addAll(getHeldResourcesForGoal(goal));
      } else {
        GoalInfo snapshot = goal.getInfo();
        for (Symbol agent : getDescendants(goal.getActor())) {
          for (Symbol resource : schedulingPolicy.requiredResources(snapshot, agent)) {
            resourceSet.add(agentTeams.get(agent).getResource(resource));
          }
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

      GoalInfo snapshot = goal.getInfo();
      for (Symbol agent : getDescendants(goal.getActor())) {
        for (Symbol resource : schedulingPolicy.heldResources(snapshot, agent)) {
          resourceSet.add(agentTeams.get(agent).getResource(resource));
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
    if (status.isTerminated() && updateType == UpdateType.REMOVED) {
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
    coordinator.run(() -> updatePendingGoal(g, updateType));
  }

  private void updatePendingGoal(Goal g, UpdateType updateType) {
    log.trace("[onPendingGoalUpdated] {}", g);

    //If a pending goal was newly added, check if it should be forwarded straight to execution or left in the queue
    if (updateType == UpdateType.ADDED) {
      refreshPendingOrder();
      if (expireUnstartedGoal(g)) {
        return;
      }
      PendingGoal entry = getPendingGoal(g.getId());
      if (entry == null || g.getStatus().isTerminated() || cancelingGoals.contains(g) || shuttingDown) {
        return;
      }
      int currentIndex = getIndexOfPendingGoal(entry);
      log.trace("[onPendingGoalUpdated] pending goal has been added");
      synchronized (resourceLock) {
        //Gather required resources for the added goal
        log.trace("[onPendingGoalUpdated] have resourceLock");
        //Gather active goals which are occupying resources necessary for this one
        Set<Goal> conflictingGoals = getResourceConflictingActiveGoals(g);
        //Transfer to active immediately if the newly submitted goal:
        //1. Does not conflict with any currently active goal
        if (conflictingGoals.isEmpty()) {
          log.trace("[onPendingGoalUpdated] no conflicting active goals");
          //2. Does not share relevant resources with any higher priority goal currently in the queue
          if (!resourceConflictInPending(g, currentIndex)) {
            log.debug("[onPendingGoalUpdated] no conflicting goals, transferring to active");
            if (lockResources(g)) {
              transferGoalToActive(g);
            }
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
    if (schedulingPolicy.queueConflicts) {
      if (g.getStatus() == GoalStatus.PENDING) {
        coordinator.defer(() -> g.notifyIfStatus(GoalStatus.PENDING));
      }
      return;
    }
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
    cancelingGoals.add(g);
    coordinator.defer(() -> g.setFailureStatus(GoalStatus.FAILED, justification));
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
    return submitGoal(g, null);
  }

  private Goal submitGoal(Goal g, ScheduledGoal schedule) {
    log.info("[submitGoal] submitting goal {} with exec type {}, priority {}, tier {}", g, g.getExecutionType(), g.getPriority(), g.getPriorityTier());

    CompletableFuture<Void> submission = new CompletableFuture<>();
    int admission = coordinator.call(() -> {
      if (schedule != null && scheduledAdmissionClosed(schedule)) {
        return -1;
      }
      if (registerGoal(g)) {
        submissionsInFlight.put(g, submission);
        return 1;
      }
      if (g.isLearningGoal() && learningGoals.get(g.getId()) == g) {
        checkRecoveryReady();
        learningGoals.remove(g.getId(), g);
        submissionsInFlight.put(g, submission);
        return 2;
      }
      return 0;
    });
    if (admission == -1) {
      g.setStatus(GoalStatus.CANCELED);
      return g;
    }
    if (admission == 0) {
      log.debug("[submitGoal] goal {} was already submitted", g.getId());
      return g;
    }
    try {
      beginManagerOperation();
      checkpointRecovery(false);
      return submitRegisteredGoal(g, admission == 2);
    } catch (UncheckedIOException e) {
      g.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
              Factory.createPredicate("goalRecoveryUnavailable", g.getGoalPredicate())));
      throw e;
    } finally {
      try {
        coordinator.run(() -> submissionsInFlight.remove(g, submission));
      } finally {
        try {
          submission.complete(null);
          trimGoalHistory();
        } finally {
          endManagerOperation();
        }
      }
    }
  }

  private Goal submitRegisteredGoal(Goal g, boolean learningContinuation) {
    if (g.getStatus().isTerminated()) {
      transferGoalToPastGoals(g);
      return g;
    }

    Symbol untypedActor = g.getActor().toUntyped();
    if (getAgentTeam(untypedActor) == null) {
      log.error("[submitGoal] actor {} for goal {} not found in the agent hierarchy. Not executing. ", untypedActor, g);
      Justification justification = new ConditionJustification(false, Factory.createPredicate("actorInHierarchy", untypedActor));
      g.setFailureStatus(GoalStatus.FAILED, justification);
      return g;
    }

    if (g.getScheduledState() == 1) {
      // check if goal is a scheduled goal, and handle
      scheduledGoalManager.executeScheduledGoal((ScheduledGoal) g);
      return g;
    } else if (g.getScheduledState() == 0 && !learningContinuation) {
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

  private boolean registerGoal(Goal goal) {
    if (!coordinator.isOwner()) {
      return coordinator.call(() -> registerGoal(goal));
    }
    if (shuttingDown) {
      throw new RejectedExecutionException("Goal manager is shutting down");
    }
    if (registeredGoals.containsKey(goal.getId())) {
      return false;
    }
    checkAdmissionCapacity(goal);
    registeredGoals.put(goal.getId(), goal);
    try {
      goal.setLifecycleListener(this);
    } catch (IllegalStateException e) {
      registeredGoals.remove(goal.getId(), goal);
      throw e;
    }
    if (notifier != null) {
      goal.addListener(notifier);
    }
    long deadline = goal.getInfo().startDeadline;
    if (deadline > 0) {
      if (!expireUnstartedGoal(goal)) {
        armDeadlineTimer(goal);
      }
    }
    return true;
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
    if (!awaitScheduledContinuation(gid, null)) {
      Goal goal = getGoal(gid);
      return goal == null ? GoalStatus.PENDING : goal.getStatus();
    }

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
    long startTime = System.nanoTime();
    if (!awaitScheduledContinuation(gid, millis)) {
      Goal goal = getGoal(gid);
      return goal == null ? GoalStatus.PENDING : goal.getStatus();
    }
    millis = Math.max(0, millis - TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));
    startTime = System.nanoTime();
    if (!joinOnPendingGoal(gid, millis)) {
      log.trace("[joinOnGoal] timed out for gid {}", gid);
      // returned while waiting for pending goal to no longer be pending
      return GoalStatus.PENDING;
    }
    millis = Math.max(0, millis - TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));

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
    if (!coordinator.isOwner()) {
      coordinator.run(() -> addPendingGoal(pg));
      return;
    }
    log.trace("[addPendingGoal] in method with goal {}", pg.getGoal().getId());
    synchronized (pendingGoalsLock) {
      log.trace("[addPendingGoal] have pendingGoalsLock");
      if (pg.getGoal().getStatus().isTerminated()) {
        transferGoalToPastGoals(pg.getGoal());
        pg.notifyOfNoLongerPending();
        return;
      }
      pendingGoals.add(pg);
      pendingSince.putIfAbsent(pg.getGoal(), schedulingTimeNanos());
      refreshPendingOrder();

      if (pg.getPreviousAIFuture() != null) {
        coordinator.defer(() -> onActiveGoalUpdated(pg.getGoal(), pg.getGoal().getStatus(), UpdateType.REMOVED));
      }
      int index = getIndexOfPendingGoal(pg); //not ideal, see if we want to change UI hook signature
      //Notify of new goal submission
      coordinator.defer(() -> onPendingGoalUpdated(pg.getGoal(), index, UpdateType.ADDED));
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
    if (!coordinator.isOwner()) {
      coordinator.run(() -> supersedeGoals(newGoal, activeGoals));
      return;
    }
    for (Goal activeGoal : activeGoals) {
      if (activeGoal.getStatus().isTerminated() || startingGoals.containsKey(activeGoal)
              || preemptions.containsKey(activeGoal) || resumingGoals.contains(activeGoal)
              || cancelingGoals.contains(activeGoal)) {
        continue;
      }
      preemptions.put(activeGoal, activeGoal.getStatus());
      coordinator.defer(() -> {
        boolean accepted;
        try {
          accepted = activeGoal.getPriorityTier() == PriorityTier.KIOSK
                  ? cancelActiveGoal(activeGoal)
                  : activeGoal.getStatus() == GoalStatus.SUSPENDED || suspendActiveGoal(activeGoal);
        } catch (RuntimeException | Error e) {
          coordinator.run(() -> preemptions.remove(activeGoal));
          log.error("Could not interrupt goal {} for preemption", activeGoal, e);
          throw e;
        }
        coordinator.run(() -> {
          if (!accepted) {
            preemptions.remove(activeGoal);
          } else {
            acknowledgePreemption(activeGoal);
          }
        });
      });
    }
  }

  private void acknowledgePreemption(Goal goal) {
    GoalStatus previous = preemptions.get(goal);
    if (previous == null || goal.getStatus() != GoalStatus.SUSPENDED
            || goal.getPriorityTier() == PriorityTier.KIOSK) {
      return;
    }
    preemptions.remove(goal);
    synchronized (pendingGoalsLock) {
      synchronized (resourceLock) {
        synchronized (goalsLock) {
          Future future = removeActiveGoal(goal);
          if (future == null) {
            return;
          }
          unlockResources(goal);
          PendingGoal pending = new PendingGoal(goal);
          pending.setPreviousAIFuture(future);
          pending.setPreviousStatus(previous);
          addPendingGoal(pending);
        }
      }
    }
    activateValidPendingGoals();
  }

  /**
   * Transfer a goal from pending goals to active goals, and start execution of the goal. Note that the goal is not
   * guaranteed to be in the active goals when this method returns. If the goal fails, for instance, the goal
   * will automatically be moved to the past goals.
   *
   * @param goal the goal to transfer and execute
   */
  protected void transferGoalToActive(Goal goal) {
    if (!coordinator.isOwner()) {
      coordinator.run(() -> transferGoalToActive(goal));
      return;
    }
    if (shuttingDown || expireUnstartedGoal(goal)) {
      return;
    }
    log.debug("[transferGoalToActive] in method with goal: " + goal);

    synchronized (pendingGoalsLock) {
      log.trace("[transferGoalToActive] have pendingGoalsLock");
      PendingGoal existing = getPendingGoal(goal.getId());
      if (existing == null || goal.getStatus().isTerminated()) {
        return;
      }
      if (!lockResources(goal)) {
        return;
      }
      int index = getIndexOfPendingGoal(existing);
      PendingGoal pg = removePendingGoal(goal);
      if (pg == null) {
        log.error("[transferGoalToActive] attempting to submit goal which doesn't exist in pending queue");
        unlockResources(goal);
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
          coordinator.defer(() -> onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.ADDED));
        } else {
          if (pg.getPreviousStatus() == GoalStatus.ACTIVE || pg.getPreviousStatus() == GoalStatus.PENDING) {
            log.debug("[transferGoalToActive] Resuming goal with previous AI");
            resumingGoals.add(goal);
            coordinator.defer(() -> {
              performResume(goal);
              onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.ADDED);
            });
          } else {
            log.debug("[transferGoalToActive] Leaving goal with current status {}", goal.getStatus());
            coordinator.defer(() -> onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.ADDED));
          }
        }
        coordinator.defer(() -> onPendingGoalUpdated(goal, index, UpdateType.REMOVED));
        return;
      }

      StartingGoal starting = new StartingGoal();
      startingGoals.put(goal, starting);
      executionAttempts.put(goal, starting);
      addActiveGoal(goal, starting.completion);
      coordinator.defer(() -> {
        try {
          beginManagerOperation();
          if (!goal.getStatus().isTerminated()) {
            executeGoal(goal);
          }
        } catch (RuntimeException | Error e) {
          log.error("Could not start execution of goal {}", goal, e);
          goal.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
                  Factory.createPredicate("executionStartupFailed", goal.getGoalPredicate())));
          throw e;
        } finally {
          try {
            finishStarting(goal, starting);
          } finally {
            try {
              pg.notifyOfNoLongerPending();
              onPendingGoalUpdated(goal, index, UpdateType.REMOVED);
            } finally {
              try {
                starting.startupFinished.complete(null);
                trimGoalHistory();
              } finally {
                endManagerOperation();
              }
            }
          }
        }
      });
    }
  }

  private void finishStarting(Goal goal, StartingGoal starting) {
    boolean cancel = coordinator.call(() -> {
      startingGoals.remove(goal, starting);
      return starting.cancelRequested && !starting.dispatched && !goal.getStatus().isTerminated();
    });
    if (cancel) {
      goal.cancel();
    }
    reconsiderPendingGoals();
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

  private boolean lockResources(Goal g) {
    synchronized (resourceLock) {
      if (cancelingGoals.contains(g) || g.getStatus().isTerminated()) {
        return false;
      }
      if (resourceAllocations.containsKey(g)) {
        return true;
      }
      if (!getResourceConflictingActiveGoals(g).isEmpty()) {
        return false;
      }
      ResourceClaims claims = getResourceClaims(g);
      for (Resource res : claims.held) {
        res.setHolder(g);
      }
      for (Resource res : claims.used) {
        res.addUser(g);
      }
      resourceAllocations.put(g, claims);
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

    return agentTeam.removeGoal(g);
  }

  private boolean unlockResources(Goal g) {
    synchronized (resourceLock) {
      ResourceClaims claims = resourceAllocations.remove(g);
      if (claims == null) {
        claims = getResourceClaims(g);
      }
      for (Resource res : claims.held) {
        if (res.getHolder() == g) {
          res.releaseHolder();
        }
      }
      for (Resource res : claims.used) {
        res.removeUser(g);
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
    Justification constraintCheck = root.getConstraints().verifyConstraints(goal.getGoalStatePredicate(), stateMachine);

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
      log.warn("[executeGoal] Goal " + goal + " is not permissible.");
      goal.setFailureStatus(GoalStatus.FAILED, constraintCheck);
    }

    log.warn("[executeGoal] Failed to add goal! {}", goal);
    return false;
  }

  /**
   * Submits the supplied actionInterpreter to this class' ExecutorService while doing associated bookkeeping
   */
  private void startActionInterpreter(ActionInterpreter ai) {
    log.debug("Starting interpreter for goal " + ai.getGoal() + "...");
    Goal goal = ai.getGoal();
    StartingGoal starting = coordinator.call(() -> startingGoals.get(goal));
    FutureTask<ActionStatus> aiFuture = new FutureTask<>(ai) {
      @Override
      protected void done() {
        if (starting != null) {
          try {
            starting.completion.complete(get());
          } catch (CancellationException e) {
            starting.completion.cancel(false);
          } catch (ExecutionException e) {
            starting.completion.completeExceptionally(e.getCause());
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            starting.completion.completeExceptionally(e);
          } finally {
            trimGoalHistory();
          }
        }
      }
    };
    boolean dispatch = coordinator.call(() -> {
      synchronized (pendingGoalsLock) {
        synchronized (resourceLock) {
          synchronized (goalsLock) {
            if (expireUnstartedGoal(goal) || shuttingDown || cancelingGoals.contains(goal)
                    || recoveryFailure != null || !recoveryRecords.isEmpty()
                    || goal.getStatus().isTerminated() || (starting != null && starting.cancelRequested)) {
              return false;
            }
            if (starting != null) {
              starting.dispatched = true;
              startingGoals.remove(goal, starting);
            }
            cancelDeadlineTimer(goal);
            addActiveGoal(goal, aiFuture);
            return true;
          }
        }
      }
    });
    if (!dispatch) {
      try {
        if (!goal.getStatus().isTerminated()) {
          goal.setStatus(GoalStatus.CANCELED);
        }
      } finally {
        aiFuture.cancel(false);
      }
      return;
    }
    try {
      onActiveGoalUpdated(goal, GoalStatus.ACTIVE, UpdateType.ADDED);
      executor.execute(aiFuture);
    } catch (RuntimeException | Error e) {
      log.error("Could not dispatch interpreter for goal {}", goal, e);
      try {
        goal.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
                Factory.createPredicate(e instanceof RejectedExecutionException
                        ? "interpreterRejected" : "executionStartupFailed", goal.getGoalPredicate())));
      } finally {
        aiFuture.cancel(false);
      }
      throw e;
    }
    //updatePriorities();
  }

  public void interruptWaitForNextGoalStatusUpdate(long gid) {
    goalStatusUpdateLock.lock();
    try {
      GoalWaitState state = goalWaitStates.computeIfAbsent(gid, key -> new GoalWaitState());
      if (state.waiters == 0) {
        state.pendingInterrupt = true;
      } else {
        state.interruptVersion++;
      }
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
    validateWaitArguments(0, timeout);
    Goal goal = getGoal(gid);

    if (goal == null) {
      return null;
    }
    GoalInfo snapshot = goal.getInfo();
    if (startStatus == null && snapshot.status.isTerminated()) {
      return null;
    }
    return legacyWaitResult(awaitGoalUpdate(goal, snapshot.statusVersion, timeout,
            startStatus == null ? snapshot.status : startStatus));
  }

  /**
   * Wait until a particular goal has its goal status updated to any new value.
   *
   * @param goal
   * @param timeout
   * @return The GoalStatus after the soonest update occurred or the timeout duration elapsed without an update
   */
  protected GoalStatus waitForNextGoalStatusUpdate(Goal goal, long timeout, GoalStatus startStatus) {
    GoalInfo snapshot = goal.getInfo();
    return legacyWaitResult(awaitGoalUpdate(goal, snapshot.statusVersion, timeout,
            startStatus == null ? snapshot.status : startStatus));
  }

  /**
   * Wait for a transition after afterVersion. Zero timeout waits indefinitely; a terminal goal never blocks.
   */
  public GoalWaitResult waitForGoalUpdate(long gid, long afterVersion, long timeout) {
    validateWaitArguments(afterVersion, timeout);
    Goal goal = getGoal(gid);
    if (goal == null) {
      return new GoalWaitResult(GoalWaitResult.Outcome.UNKNOWN_GOAL, null);
    }
    return awaitGoalUpdate(goal, afterVersion, timeout, null);
  }

  /**
   * Capture membership and resource blockers under the same lock order used by scheduling.
   * Status metadata is a versioned snapshot; interpreter transitions can still be in flight.
   */
  public GoalSchedulingExplanation explainGoalScheduling(long gid) {
    if (!coordinator.isOwner()) {
      return coordinator.call(() -> explainGoalScheduling(gid));
    }
    synchronized (pendingGoalsLock) {
      synchronized (resourceLock) {
        synchronized (goalsLock) {
          Goal goal = getGoal(gid);
          if (goal == null) {
            return new GoalSchedulingExplanation(gid, getClass().getName(), null,
                    GoalSchedulingExplanation.Location.UNKNOWN, -1,
                    List.of(GoalSchedulingExplanation.Reason.UNKNOWN_GOAL), List.of(), List.of());
          }
          GoalInfo snapshot = goal.getInfo();
          PendingGoal pending = getPendingGoal(gid);
          int position = pending == null ? -1 : getIndexOfPendingGoal(pending);
          GoalSchedulingExplanation.Location location;
          if (pending != null) {
            location = GoalSchedulingExplanation.Location.PENDING;
          } else if (getActiveGoal(gid) != null) {
            location = GoalSchedulingExplanation.Location.ACTIVE;
          } else if (pastGoals.contains(goal)) {
            location = GoalSchedulingExplanation.Location.PAST;
          } else if (learningGoals.containsKey(gid)) {
            location = GoalSchedulingExplanation.Location.LEARNING;
          } else if (goal.isScheduled() && scheduledGoalManager != null
                  && scheduledGoalManager.getScheduledGoal(gid) != null) {
            location = GoalSchedulingExplanation.Location.SCHEDULED;
          } else {
            location = GoalSchedulingExplanation.Location.SUBMITTING;
          }

          ResourceClaims claims = resourceAllocations.get(goal);
          if (claims == null) {
            claims = pending == null ? new ResourceClaims(Set.of(), Set.of()) : getResourceClaims(goal);
          }
          List<GoalSchedulingExplanation.ResourceClaim> resources = new ArrayList<>();
          for (Resource resource : orderedResources(claims.used)) {
            resources.add(new GoalSchedulingExplanation.ResourceClaim(resourceProvider(resource),
                    resource.getName().toString(), claims.required.contains(resource), claims.held.contains(resource)));
          }
          List<GoalSchedulingExplanation.Blocker> blockers = new ArrayList<>();
          List<GoalSchedulingExplanation.Reason> reasons = new ArrayList<>();
          if (snapshot.status.isTerminated()) {
            reasons.add(location == GoalSchedulingExplanation.Location.PAST
                    ? GoalSchedulingExplanation.Reason.TERMINATED : GoalSchedulingExplanation.Reason.TERMINATING);
          } else if (pending != null) {
            List<Goal> activeBlockers = new ArrayList<>(getResourceConflictingActiveGoals(goal));
            activeBlockers.sort(Comparator.comparingLong(Goal::getId));
            for (Goal blocker : activeBlockers) {
              ResourceClaims allocation = resourceAllocations.get(blocker);
              blockers.add(explainBlocker(claims, blocker,
                      allocation == null ? getResourceClaims(blocker) : allocation,
                      GoalSchedulingExplanation.Location.ACTIVE));
            }
            if (!activeBlockers.isEmpty()) {
              reasons.add(GoalSchedulingExplanation.Reason.ACTIVE_RESOURCE_CONFLICT);
            }
            Iterator<PendingGoal> iterator = pendingGoals.descendingIterator();
            for (int i = 0; i < position && iterator.hasNext(); i++) {
              Goal blocker = iterator.next().getGoal();
              ResourceClaims other = getResourceClaims(blocker);
              if (claims.conflictsWith(other)) {
                blockers.add(explainBlocker(claims, blocker, other, GoalSchedulingExplanation.Location.PENDING));
                if (!reasons.contains(GoalSchedulingExplanation.Reason.HIGHER_PRIORITY_PENDING_CONFLICT)) {
                  reasons.add(GoalSchedulingExplanation.Reason.HIGHER_PRIORITY_PENDING_CONFLICT);
                }
              }
            }
            if (snapshot.status == GoalStatus.SUSPENDED) {
              reasons.add(GoalSchedulingExplanation.Reason.SUSPENDED);
            }
            if (blockers.isEmpty()) {
              reasons.add(GoalSchedulingExplanation.Reason.READY);
            }
          } else {
            switch (location) {
              case ACTIVE:
                if (snapshot.status == GoalStatus.SUSPENDED) {
                  reasons.add(GoalSchedulingExplanation.Reason.SUSPENDED);
                } else {
                  reasons.add(startingGoals.containsKey(goal) || snapshot.status == GoalStatus.PENDING
                          ? GoalSchedulingExplanation.Reason.STARTING : GoalSchedulingExplanation.Reason.EXECUTING);
                }
                break;
              case PAST:
                reasons.add(GoalSchedulingExplanation.Reason.TERMINATED);
                break;
              case LEARNING:
                reasons.add(GoalSchedulingExplanation.Reason.LEARNING);
                break;
              case SCHEDULED:
                reasons.add(GoalSchedulingExplanation.Reason.SCHEDULED);
                break;
              default:
                reasons.add(GoalSchedulingExplanation.Reason.SUBMITTING);
            }
          }
          return new GoalSchedulingExplanation(gid, getClass().getName(), snapshot, location,
                  position, reasons, resources, blockers, effectivePriority(goal), agingIntervalMillis);
        }
      }
    }
  }

  private GoalSchedulingExplanation.Blocker explainBlocker(ResourceClaims claims, Goal blocker,
                                                           ResourceClaims other,
                                                           GoalSchedulingExplanation.Location location) {
    List<GoalSchedulingExplanation.ResourceConflict> resources = new ArrayList<>();
    for (Resource resource : orderedResources(claims.conflictingResources(other))) {
      resources.add(new GoalSchedulingExplanation.ResourceConflict(resourceProvider(resource),
              resource.getName().toString(), claims.held.contains(resource), other.held.contains(resource)));
    }
    return new GoalSchedulingExplanation.Blocker(blocker.getInfo(), location, resources);
  }

  private static String resourceProvider(Resource resource) {
    return resource.getProvider() == null ? null : resource.getProvider().getName().toString();
  }

  private static List<Resource> orderedResources(Set<Resource> resources) {
    List<Resource> ordered = new ArrayList<>(resources);
    ordered.sort(Comparator.comparing((Resource resource) -> Objects.toString(resourceProvider(resource), ""))
            .thenComparing(resource -> resource.getName().toString()));
    return ordered;
  }

  private static void validateWaitArguments(long afterVersion, long timeout) {
    if (afterVersion < 0 || timeout < 0) {
      throw new IllegalArgumentException("Goal version and timeout must be nonnegative");
    }
  }

  private static GoalStatus legacyWaitResult(GoalWaitResult result) {
    if (result.outcome == GoalWaitResult.Outcome.UPDATED || result.outcome == GoalWaitResult.Outcome.TERMINATED) {
      return result.getGoalInfo().status;
    }
    return null;
  }

  private GoalWaitResult awaitGoalUpdate(Goal goal, long afterVersion, long timeout, GoalStatus startStatus) {
    validateWaitArguments(afterVersion, timeout);
    goalStatusUpdateLock.lock();
    GoalWaitState state = goalWaitStates.computeIfAbsent(goal.getId(), key -> new GoalWaitState());
    long interruptVersion = state.interruptVersion;
    state.waiters++;
    long remaining = TimeUnit.MILLISECONDS.toNanos(timeout);
    try {
      while (true) {
        GoalInfo snapshot = goal.getInfo();
        if (Thread.currentThread().isInterrupted()) {
          return new GoalWaitResult(GoalWaitResult.Outcome.INTERRUPTED, snapshot);
        }
        if (snapshot.status.isTerminated()) {
          return new GoalWaitResult(GoalWaitResult.Outcome.TERMINATED, snapshot);
        }
        if (startStatus == null ? snapshot.statusVersion > afterVersion : snapshot.status != startStatus) {
          return new GoalWaitResult(GoalWaitResult.Outcome.UPDATED, snapshot);
        }
        if (state.pendingInterrupt || state.interruptVersion != interruptVersion) {
          state.pendingInterrupt = false;
          return new GoalWaitResult(GoalWaitResult.Outcome.INTERRUPTED, snapshot);
        }
        if (timeout == 0) {
          goalStatusUpdateCondition.await();
        } else {
          if (remaining <= 0) {
            return new GoalWaitResult(GoalWaitResult.Outcome.TIMED_OUT, snapshot);
          }
          remaining = goalStatusUpdateCondition.awaitNanos(remaining);
        }
      }
    } catch (InterruptedException e) {
      log.error("Error waiting on new status for goal {}", goal, e);
      Thread.currentThread().interrupt();
      return new GoalWaitResult(GoalWaitResult.Outcome.INTERRUPTED, goal.getInfo());
    } finally {
      if (--state.waiters == 0 && !state.pendingInterrupt) {
        goalWaitStates.remove(goal.getId(), state);
      }
      goalStatusUpdateLock.unlock();
    }
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
      while (goal.getStatus() != status && !goal.getStatus().isTerminated()) {
        goalStatusUpdateCondition.await();
      }
    } catch (InterruptedException e) {
      log.error("Error waiting for goal {} with status {}", goal, status);
      Thread.currentThread().interrupt();
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
    trackManagerOperation();
    try {
      handleGoalStatusChanged(info);
    } finally {
      endManagerOperation();
    }
  }

  private void handleGoalStatusChanged(GoalInfo info) {
    log.debug("[goalStatusChanged] goal: {} status: {}", info.goal, info.status);

    // notify everyone waiting on goal status notifications
    goalStatusUpdateLock.lock();
    try {
      goalStatusUpdateCondition.signalAll();
    } finally {
      goalStatusUpdateLock.unlock();
    }

    coordinator.run(() -> {
      Goal goal = getGoal(info.gid);
      if (goal == null) {
        if (info.status.isTerminated()) {
          log.warn("[goalStatusChanged] Could not find goal: {}", info.goal);
        }
        return;
      }
      if (goal.getStatus().isTerminated()) {
        transferGoalToPastGoals(goal);
      } else if (goal.getInfo().statusVersion == info.statusVersion) {
        acknowledgePreemption(goal);
      }
    });
    if (recoveryFailure == null) {
      checkpointRecovery(false);
    }
  }

  /**
   * Searches for the supplied goal in the pending and active goal collections, moving it to pastGoals and performing
   * the relevant bookkeeping if found
   */
  protected void transferGoalToPastGoals(Goal goal) {
    if (!coordinator.isOwner()) {
      coordinator.run(() -> transferGoalToPastGoals(goal));
      return;
    }
    if (!goal.getStatus().isTerminated()) {
      return;
    }
    PendingGoal pg;
    int pendingIndex;
    boolean wasActive;
    synchronized (pendingGoalsLock) {
      if (pastGoals.contains(goal)) {
        return;
      }
      learningGoals.remove(goal.getId(), goal);
      preemptions.remove(goal);
      cancelingGoals.remove(goal);
      resumingGoals.remove(goal);
      startingGoals.remove(goal);
      cancelDeadlineTimer(goal);
      StartingGoal starting = executionAttempts.get(goal);
      if (starting != null && !starting.dispatched) {
        coordinator.defer(() -> starting.completion.complete(
                goal.getStatus() == GoalStatus.SUCCEEDED ? ActionStatus.SUCCESS
                        : goal.getStatus() == GoalStatus.CANCELED ? ActionStatus.CANCEL : ActionStatus.FAIL));
      }
      PendingGoal existing = getPendingGoal(goal.getId());
      pendingIndex = existing == null ? -1 : getIndexOfPendingGoal(existing);
      pg = removePendingGoal(goal);
      agedPriorities.remove(goal);
      synchronized (resourceLock) {
        synchronized (goalsLock) {
          wasActive = getActiveGoal(goal.getId()) != null;
          if (wasActive || resourceAllocations.containsKey(goal)) {
            unlockResources(goal);
          }
          if (wasActive) {
            removeActiveGoal(goal);
          }
          if (goal.isScheduled() && scheduledGoalManager != null) {
            scheduledGoalManager.removeGoal((ScheduledGoal) goal);
          }
          pastGoals.add(goal);
          trimGoalHistory();
        }
      }
    }
    if (pg != null) {
      pg.notifyOfNoLongerPending();
      coordinator.defer(() -> onPendingGoalUpdated(goal, pendingIndex, UpdateType.REMOVED));
    }
    if (wasActive) {
      coordinator.defer(() -> onActiveGoalUpdated(goal, goal.getStatus(), UpdateType.REMOVED));
    }
    if (maxPastGoals > 0) {
      coordinator.defer(this::trimGoalHistory);
    }
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
   * Remove a pending entry. The enclosing lifecycle transition publishes its hook after bookkeeping.
   */
  private PendingGoal removePendingGoal(Goal g) {
    log.debug("[removePendingGoal] {}", g);
    synchronized (pendingGoalsLock) {
      log.trace("[removePendingGoal] {} have pendingGoalsLock", g);
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pendingGoal = pendingGoalIterator.next();
        if (pendingGoal.getGoal().getId() == g.getId()) {
          log.trace("[removePendingGoal] {} found goal", g);
          pendingGoalIterator.remove();
          pendingSince.remove(g);
          return pendingGoal;
        }
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
    Supplier<Boolean> effect = coordinator.call(() -> {
      Goal goal = getGoal(gid);
      if (goal == null || goal.getStatus().isTerminated()) {
        log.warn("[cancelGoal] cannot find a nonterminal goal for gid {}", gid);
        return () -> false;
      }
      cancelingGoals.add(goal);
      StartingGoal starting = startingGoals.get(goal);
      if (starting != null) {
        starting.cancelRequested = true;
        return () -> true;
      }
      return () -> {
        boolean accepted;
        try {
          accepted = goal.isScheduled()
                  ? scheduledGoalManager.cancelScheduledGoal((ScheduledGoal) goal) : goal.cancel();
        } catch (RuntimeException | Error e) {
          coordinator.run(() -> cancelingGoals.remove(goal));
          throw e;
        }
        if (!accepted) {
          coordinator.run(() -> cancelingGoals.remove(goal));
        }
        return accepted;
      };
    });
    return effect.get();
  }

  /**
   * Cancel the supplied pending goal and transfer to past goals
   *
   * @param pg
   */
  private void cancelPendingGoal(PendingGoal pg) {
    cancelGoal(pg.getGoal().getId());
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
   * Cancels the upcoming pending goals at the supplied indices (Predicate in the form and(index1,index2,...))
   */
  public boolean cancelPendingGoalsByIndices(Predicate indicesPred) {
    log.debug("[cancelPendingGoalsByIndices] removing pending goals with indices: " + indicesPred);
    List<PendingGoal> entries = new ArrayList<>();
    synchronized (pendingGoalsLock) {
      List<Integer> indices = new ArrayList<>();
      for (Symbol index : indicesPred.getArgs()) {
        try {
          indices.add((int) Double.parseDouble(index.getName()));
        } catch (NumberFormatException e) {
          log.error("[cancelPendingGoalsByIndices] unexpected index {}", index, e);
        }
      }
      indices.sort(Comparator.reverseOrder());
      for (int index : indices) {
        if (index < pendingGoals.size()) {
          Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
          int i = 0;
          while (i < index) {
            pendingGoalsIterator.next();
            i++;
          }
          entries.add(pendingGoalsIterator.next());
        } else {
          log.warn("[cancelPendingGoalsByIndices] Queue is of length: " + pendingGoals.size() + ", cannot remove goal with index: " + index);
        }
      }
    }
    for (PendingGoal entry : entries) {
      cancelPendingGoal(entry);
    }
    return true;
  }

  /**
   * Cancels the upcoming scheduled goals at the supplied indices (Predicate in the form and(index1,index2,...))
   */
  public boolean cancelScheduledGoalsByIndices(Predicate indicesPred) {
    log.debug("[cancelScheduledGoalsByIndices] removing scheduled goals with indices: " + indicesPred);
    List<ScheduledGoal> scheduledGoals = getScheduledGoals();
    List<Integer> indices = new ArrayList<>();
    for (Symbol index : indicesPred.getArgs()) {
      try {
        indices.add((int) Double.parseDouble(index.getName()));
      } catch (NumberFormatException e) {
        log.error("[cancelScheduledGoalsByIndices] unexpected index {}", index, e);
      }
    }
    indices.sort(Comparator.reverseOrder());
    for (int index : indices) {
      if (index < scheduledGoals.size()) {
        cancelGoal(scheduledGoals.get(index).getId());
      } else {
        log.warn("[cancelScheduledGoalsByIndices] Have " + scheduledGoals.size() + " scheduled goals, cannot remove goal with index: " + index);
      }
    }
    return true;
  }

  /**
   * Cancel the supplied active goal and transfer to past goals
   *
   * @param goal
   */
  private boolean cancelActiveGoal(Goal goal) {
    log.debug("[cancelActiveGoal] goal: {}", goal.getGoalPredicate());
    return goal.cancel();
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
    if (coordinator.call(() -> {
      if (preemptions.containsKey(g)) {
        preemptions.put(g, GoalStatus.SUSPENDED);
        return true;
      }
      return false;
    })) {
      return true;
    }
    if (g.getStatus() == GoalStatus.PENDING) {
      log.warn("[suspendGoal] attempting to suspend a pending goal");
      return false;
    } else if (g.getStatus().isTerminated()) {
      log.warn("[suspendGoal] attempting to suspend an already terminated goal");
      return false;
    } else if (g.getStatus().equals(GoalStatus.SUSPENDED)) {
      // EW: Should this explicitly live in a separate method?
      // Functionality required for social LLM "OffContext" dialogue goal interactions - set a currently superseded
      //  pending goal to stay suspended when returning to the current collection
      boolean pending = coordinator.call(() -> {
        PendingGoal pg = getPendingGoal(gid);
        if (preemptions.containsKey(g)) {
          preemptions.put(g, GoalStatus.SUSPENDED);
          return true;
        }
        if (pg != null && pg.getPreviousAIFuture() != null) {
          pg.setPreviousStatus(GoalStatus.SUSPENDED);
          return true;
        }
        return false;
      });
      if (pending) {
        return true;
      }
      return suspendActiveGoal(g);
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
    log.trace("[suspendActiveGoal] " + goal.getGoalPredicate());
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
      boolean pending = coordinator.call(() -> {
        if (preemptions.containsKey(g)) {
          preemptions.put(g, GoalStatus.ACTIVE);
          return true;
        }
        PendingGoal pg = getPendingGoal(gid);
        if (pg == null) {
          return false;
        }
        pg.setPreviousStatus(GoalStatus.ACTIVE);
        activateValidPendingGoals();
        return true;
      });
      if (pending) {
        return true;
      }
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
    boolean accepted = coordinator.call(() -> {
      if (goal == null || goal.getStatus().isTerminated() || cancelingGoals.contains(goal)
              || resumingGoals.contains(goal) || preemptions.containsKey(goal)
              || getActiveGoal(goal.getId()) != goal) {
        return false;
      }
      resumingGoals.add(goal);
      return true;
    });
    if (!accepted) {
      return false;
    }
    return performResume(goal);
  }

  private boolean performResume(Goal goal) {
    try {
      if (!coordinator.call(() -> resumingGoals.contains(goal)
              && getActiveGoal(goal.getId()) == goal && !cancelingGoals.contains(goal)
              && !goal.getStatus().isTerminated())) {
        return false;
      }
      if (!goal.resume()) {
        return false;
      }
      onActiveGoalUpdated(goal, GoalStatus.ACTIVE, UpdateType.UPDATED);
      return true;
    } finally {
      coordinator.run(() -> resumingGoals.remove(goal));
      reconsiderPendingGoals();
    }
  }

  private void reconsiderPendingGoals() {
    for (Goal pending : getPendingGoals()) {
      Set<Goal> conflicts = coordinator.call(() -> getResourceConflictingActiveGoals(pending));
      if (conflicts.isEmpty() || shouldSupersede(pending, conflicts)) {
        onPendingGoalUpdated(pending, -1, UpdateType.ADDED);
      }
    }
  }

  <T> T coordinateSchedulingState(Supplier<T> command) {
    return coordinator.call(command);
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

  //Add current goals of the supplied agent/team and all children
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
    // add all agent's goals
    AgentTeam agentTeam = getAgentTeam(agent);
    Set<Goal> agentTeamGoals = agentTeam.getActiveGoals();
    agentTeamGoals.addAll(getScheduledGoals().stream().filter(g -> agentTeam.isMember(g.getActor())).toList()); // add agent's scheduled goals

    // filter goals
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
    synchronized (pendingGoalsLock) {
      List<Goal> goals = new ArrayList<>();
      Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.descendingIterator();
      while (pendingGoalsIterator.hasNext()) {
        goals.add(pendingGoalsIterator.next().getGoal());
      }
      return goals;
    }
  }

  /**
   * Get a copied list of the previously executed goals
   *
   * @return list of Goals
   */
  public List<Goal> getPastGoals() {
    synchronized (pastGoals) {
      return new ArrayList<>(pastGoals);
    }
  }

  /**
   * Cancel all goals currently found in the pendingGoals collection
   */
  public void cancelPendingGoals() {
    log.debug("[cancelPendingGoals] in method");
    List<Long> goalIds = new ArrayList<>();
    synchronized (pendingGoalsLock) {
      log.trace("[cancelPendingGoals] have pendingGoalsLock");
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
    log.trace("[cancelPendingGoals] released pendingGoalsLock, canceling all gather goals");
    for (Long gid : goalIds) {
      log.trace("[cancelPendingGoals] canceling {}", gid);
      cancelGoal(gid);
    }
  }

  /**
   * Cancel all goals currently found in either the pending or active Goals collections
   */
  public void cancelCurrentGoals() {
    log.debug("[cancelCurrentGoals] in method");
    //synchronized (pendingGoalsLock) {
    //  log.trace("[cancelCurrentGoals] have pendingGoalsLock");
    //  synchronized (goalsLock) {
    //    log.trace("[cancelCurrentGoals] have goalsLock");
        scheduledGoalManager.cancelScheduledGoals();
        cancelPendingGoals();
        cancelActiveGoals();
    //  }
    //}
    //log.trace("[cancelCurrentGoals] released locks");
  }

  /**
   * Cancel all goals currently found in the activeGoals colection
   */
  public void cancelActiveGoals() {
    Set<Goal> goalsToCancel = new HashSet<>();
    synchronized (goalsLock) {
      for (Symbol agentTeam : agentTeams.keySet()) {
        for (Goal g : agentTeams.get(agentTeam).getActiveGoals()) {
          if (!g.getGoalStatePredicate().getName().equals("listen")) {
            goalsToCancel.add(g);
          }
        }
      }
    }
    for (Goal g : goalsToCancel) {
      cancelGoal(g.getId());
    }
  }

  public List<Long> cancelSystemGoals() {
    return cancelSystemGoals(rootAgent);
  }

  public void cancelScheduledGoals() {
    scheduledGoalManager.cancelScheduledGoals();
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

  public List<Goal> getSystemGoals() {
    return getSystemGoals(rootAgent);
  }

  public List<Goal> getSystemGoals(Symbol actor) {
    return getSystemGoalsHelper(actor.toUntyped(), new ArrayList<>());
  }

  private List<Goal> getSystemGoalsHelper(Symbol actor, List<Goal> goals) {
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      return goals;
    }

    for (Goal g : agentTeam.getActiveGoals()) {
      if (g.getPriorityTier() != PriorityTier.SKIPPENDING) {
        goals.add(g);
      }
    }

    for (Symbol member : agentTeam.getMemberNames()) {
      goals.addAll(getSystemGoals(member));
    }

    return goals;
  }

  public Goal getNextGoal() {
    log.trace("[getNextGoalPredicate] in method");
    synchronized (pendingGoalsLock) {
      log.trace("[getNextGoalPredicate] have pendingGoalsLock");
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      if (pendingGoalIterator.hasNext()) {
        log.trace("[getNextGoalPredicate] releasing pendingGoalsLock");
        return pendingGoalIterator.next().getGoal();
      } else {
        log.trace("[getNextGoalPredicate] releasing pendingGoalsLock");
        return null;
      }
    }
  }

  public Goal getNextGoal(Symbol agent) {
    log.trace("[getNextGoalPredicate] {}", agent);
    agent = agent.toUntyped();
    synchronized (pendingGoalsLock) {
      log.trace("[getNextGoalPredicate] {} have pendingGoalsLock", agent);
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pg = pendingGoalIterator.next();
        if (agent.equals(pg.getGoal().getActor().toUntyped())) {
          log.trace("[getNextGoalPredicate] {} releasing pendingGoalsLock", agent);
          return pg.getGoal();
        }
      }
    }
    log.trace("[getNextGoalPredicate] {} releasing pendingGoalsLock", agent);
    return null;
  }

  /**
   * Get the goal for a particular goal ID. This checks pending, current, and past goals.
   *
   * @param gid goal id.
   * @return the goal corresponding to the ID.
   */
  public Goal getGoal(long gid) {
    log.trace("[getGoal] {}", gid);
    Goal registered = registeredGoals.get(gid);
    if (registered != null) {
      return registered;
    }
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
      g = scheduledGoalManager == null ? null : scheduledGoalManager.getScheduledGoal(gid);
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
   * Get a copied list of all scheduled goals which have either yet to be submitted, or are currently executing for the
   *  supplied agentTeam
   *
   * @return shallow copy of scheduled goals
   */
  public List<ScheduledGoal> getScheduledGoals(Symbol agentTeam) {
    return scheduledGoalManager.getScheduledGoals(agentTeam);
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
   * Comparison method determining the order goals get placed into the pendingGoals queue.
   */
  private int compareGoal(PriorityTier pt1, long p1, long gid1, PriorityTier pt2, long p2, long gid2) {
    return SchedulingPolicy.comparePriority(pt1, p1, gid1, pt2, p2, gid2);
  }

  /**
   * Compare the priority of the supplied goal against the supplied priority/priorityTier values
   */
  public boolean hasGreaterPriority(long goalId, PriorityTier priorityTier, long priority) {
    Goal g = getGoal(goalId);
    if (g == null) {
      return true;
    }

    return compareGoal(g.getPriorityTier(), g.getPriority(), g.getId(), priorityTier, priority, g.getId()+1) > 0;
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
            removed = learnedActions.removeIf(a -> a.getMethodName().equals(action.getMethodName()) && a.getRoles().size() == action.getRoles().size());
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
          if (entries == null) {
            throw new IllegalArgumentException("Invalid learned actions file: " + learnedActionsFile);
          }
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
  public List<ActionDBEntry> getLearnedActions() {
    return loadLearnedActions(false);
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
        AgentTeam learningTeam = agentTeam;
        boolean accepted = coordinator.call(() -> {
          if (g.getStatus().isTerminated()) {
            return false;
          }
          learningGoals.put(g.getId(), g);
          return true;
        });
        if (accepted && !g.getStatus().isTerminated()) {
          learningTeam.addLearningGoal(g);
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
  @OnInterrupt(onCancelServiceCall = "endFreeze(?actor)", onSuspendServiceCall = "endFreeze(?actor)")
  public void freeze(Symbol actor) {
    log.debug("[freeze] {}", actor);
    actor = actor.toUntyped();
    //Set the AgentTeam and all children as 'frozen'
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.warn("[freeze] supplied agent is not found in the hierarchy: {}", actor);
      return;
    }

    //Doesn't actually do much of anything anymore, could remove. Only used for error reporting.
    agentTeam.freeze();

    //Block until this AgentTeam is unfrozen
    Lock freezeLock;
    Condition freezeLockCondition;
    boolean alreadyFrozen = freezeLocks.containsKey(actor);
    if (alreadyFrozen) {
      log.warn("[freeze] AgentTeam {} is already frozen, joining on existing lock", actor);
      freezeLock = freezeLocks.get(actor);
      freezeLockCondition = freezeConditions.get(actor);
    } else {
      freezeLock = new ReentrantLock();
      freezeLockCondition = freezeLock.newCondition();
      freezeLocks.put(actor, freezeLock);
      freezeConditions.put(actor, freezeLockCondition);
    }

    freezeLock.lock();
    try {
      log.trace("[freeze] {} waiting on condition ...", actor);
      freezeLockCondition.await();
    } catch (InterruptedException e) {
      log.error("[freeze]", e);
    } finally {
      freezeLock.unlock();
    }
    log.trace("[freeze] {}, done waiting on condition", actor);

    if (!alreadyFrozen) {
      freezeLocks.remove(actor);
      freezeConditions.remove(actor);
    }
  }

  /**
   * Ends the frozen status for the supplied agent/team
   */
  @Action
  @TRADEService
  public void endFreeze(Symbol actor) {
    log.debug("[endFreeze] {}", actor);
    actor = actor.toUntyped();
    //Unfreeze the AgentTeam and all children
    AgentTeam agentTeam = getAgentTeam(actor);
    if (agentTeam == null) {
      log.warn("[endFreeze] supplied agent is not found in the hierarchy: {}", actor);
      return;
    }

    //Unblock freeze behavior for this AgentTeam
    Lock freezeLock = freezeLocks.get(actor);
    Condition freezeCondition = freezeConditions.get(actor);
    if (freezeCondition == null || freezeLock == null) {
      if (agentTeam.isFrozen()) {
        log.warn("[endFreeze] Cannot end freeze for a child of a frozen AgentTeam. The original frozen ancestor of {} must be unfrozen", actor);
      } else {
        log.warn("[endFreeze] Agent {} is not frozen", actor);
      }
      return;
    }

    agentTeam.unfreeze();

    log.trace("[endFreeze] {}, getting freeze lock ...", actor);
    freezeLock.lock();
    try {
      log.trace("[endFreeze] {}, obtained lock, signalling condition", actor);
      freezeCondition.signalAll();
    } finally {
      freezeLock.unlock();
    }
    log.trace("[endFreeze] {}, signalled condition", actor);
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

  /** Zero leaves the corresponding bound disabled. Existing work is never canceled to meet a new limit. */
  public void configureGoalLimits(int managedGoals, int pastGoals) {
    if (managedGoals < 0 || pastGoals < 0) {
      throw new IllegalArgumentException("Goal limits must be nonnegative");
    }
    coordinator.run(() -> {
      if (shuttingDown) {
        throw new RejectedExecutionException("Goal manager is shutting down");
      }
      maxManagedGoals = managedGoals;
      maxPastGoals = pastGoals;
      trimGoalHistory();
    });
  }

  private boolean hasUnfinishedWork(Goal goal) {
    StartingGoal attempt = executionAttempts.get(goal);
    return !goal.getStatus().isTerminated() || submissionsInFlight.containsKey(goal)
            || scheduledContinuations.containsKey(goal.getId())
            || (attempt != null && (!attempt.startupFinished.isDone() || !attempt.completion.isDone()));
  }

  private void checkAdmissionCapacity(Goal candidate) {
    checkRecoveryReady();
    if (maxManagedGoals == 0) {
      return;
    }
    Set<Goal> occupied = new HashSet<>(continuationGoals.values());
    registeredGoals.values().stream().filter(this::hasUnfinishedWork).forEach(occupied::add);
    occupied.remove(candidate);
    if (occupied.size() >= maxManagedGoals) {
      log.warn("Goal admission limit {} reached; rejecting goal {}", maxManagedGoals, candidate.getId());
      throw new RejectedExecutionException("Goal admission limit reached: " + maxManagedGoals);
    }
  }

  private void checkRecoveryReady() {
    if (recoveryFailure != null || !recoveryRecords.isEmpty()) {
      log.warn("Goal admission blocked by unreconciled or unavailable recovery state");
      throw new RejectedExecutionException("Goal recovery requires operator reconciliation: " + recoveryFailure);
    }
  }

  private void trimGoalHistory() {
    if (maxPastGoals == 0) {
      return;
    }
    if (!coordinator.isOwner()) {
      try {
        coordinator.run(this::trimGoalHistory);
      } catch (RejectedExecutionException e) {
        if (!shuttingDown) {
          throw e;
        }
        log.debug("History cleanup completed after manager shutdown");
      }
      return;
    }
    List<Goal> history;
    synchronized (pastGoals) {
      history = new ArrayList<>(pastGoals);
    }
    history.sort(Comparator.comparingLong(Goal::getEndTime).thenComparingLong(Goal::getId));
    for (Goal goal : history) {
      if (pastGoals.size() <= maxPastGoals) {
        break;
      }
      if (!hasUnfinishedWork(goal)) {
        pastGoals.remove(goal);
        registeredGoals.remove(goal.getId(), goal);
        executionAttempts.remove(goal);
        if (rootContext != null && goal.getRootContext() != null) {
          Context context = goal.getRootContext();
          coordinator.defer(() -> rootContext.discardTerminatedChild(context));
        }
      }
    }
  }

  private void beginManagerOperation() {
    trackManagerOperation();
    if (shutdownEscalated) {
      Thread.currentThread().interrupt();
    }
  }

  private void endManagerOperation() {
    operationThreads.computeIfPresent(Thread.currentThread(), (thread, depth) -> depth.decrementAndGet() == 0 ? null : depth);
    synchronized (operationMonitor) {
      operationMonitor.notifyAll();
    }
  }

  private void trackManagerOperation() {
    if (!coordinator.isOwner()) {
      operationThreads.computeIfAbsent(Thread.currentThread(), thread -> new AtomicInteger()).incrementAndGet();
    }
  }

  public void pruneOldData() {
    log.debug("Pruning...");

    // prune old goals (which have references to context tree)
    long pruneTime = System.currentTimeMillis() - historyLength;

    try {
      // must synchronize explicitly while iterating over synchronizedSet
      coordinator.run(() -> {
        synchronized (pastGoals) {
          pastGoals.removeIf(pastGoal -> {
            if (hasUnfinishedWork(pastGoal)) {
              return false;
            }
            if (pastGoal.getStatus().isTerminated() && pastGoal.getEndTime() < pruneTime) {
              registeredGoals.remove(pastGoal.getId(), pastGoal);
              executionAttempts.remove(pastGoal);
              return true;
            }
            return false;
          });
        }
      });

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
   *
   * @return
   */
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

  public void setGoalManagerNotifier(GoalManagerNotifier notifier) {
    this.notifier = Objects.requireNonNull(notifier);
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

  /** Configure once, before admission. Recovered records block all new work until explicitly reconciled. */
  public void configureGoalRecovery(String filename) {
    if (filename == null || filename.isBlank()) {
      throw new IllegalArgumentException("Goal recovery filename must not be blank");
    }
    GoalRecoveryStore store = new GoalRecoveryStore(Path.of(filename));
    coordinator.run(() -> {
      if (shuttingDown || recoveryStore != null || !registeredGoals.isEmpty() || !continuationGoals.isEmpty()) {
        throw new IllegalStateException("Configure recovery once, before goal admission");
      }
      recoveryStore = store;
      recoveryFailure = "Recovery initialization in progress";
    });
    try {
      store.acquireWriter();
      List<GoalRecoveryRecord> recovered = store.load();
      coordinator.run(() -> {
        recoveryRecords.addAll(recovered);
        recoveryFailure = null;
      });
      checkpointRecovery(false);
    } catch (IOException e) {
      try {
        store.close();
      } catch (IOException closeFailure) {
        e.addSuppressed(closeFailure);
      }
      recoveryFailure = e.toString();
      log.error("Goal recovery initialization failed; admission remains closed", e);
      throw new UncheckedIOException(e);
    }
  }

  public List<GoalRecoveryRecord> getGoalRecoveryRecords() {
    return coordinator.call(() -> List.copyOf(recoveryRecords));
  }

  public boolean acknowledgeRecoveredGoal(String sessionId, long goalId) {
    Objects.requireNonNull(sessionId);
    boolean removed = coordinator.call(() -> {
      if (shuttingDown || recoveryFailure != null) {
        throw new IllegalStateException("Goal recovery is not available for reconciliation");
      }
      return recoveryRecords.removeIf(record -> record.sessionId.equals(sessionId) && record.goalId == goalId);
    });
    if (removed) {
      checkpointRecovery(false);
    } else {
      log.warn("No recovered goal matched session {} and goal {}", sessionId, goalId);
    }
    return removed;
  }

  private void checkpointRecovery(boolean cleanShutdown) {
    GoalRecoveryStore store = recoveryStore;
    if (store == null) {
      return;
    }
    if (coordinator.isOwner()) {
      coordinator.defer(() -> checkpointRecovery(cleanShutdown));
      return;
    }
    if (recoveryFailure != null) {
      throw new UncheckedIOException(new IOException("Goal recovery unavailable: " + recoveryFailure));
    }
    try {
      store.save(() -> coordinator.call(() -> {
        List<GoalRecoveryRecord> records = new ArrayList<>(recoveryRecords);
        for (Goal goal : registeredGoals.values()) {
          GoalInfo snapshot = goal.getInfo();
          if (!snapshot.status.isTerminated()) {
            ResourceClaims acquired = resourceAllocations.get(goal);
            List<GoalSchedulingExplanation.ResourceClaim> claims = new ArrayList<>();
            if (acquired != null) {
              for (Resource resource : orderedResources(acquired.used)) {
                claims.add(new GoalSchedulingExplanation.ResourceClaim(resourceProvider(resource),
                        resource.getName().toString(), acquired.required.contains(resource), acquired.held.contains(resource)));
              }
            }
            String predicate = goal instanceof ScheduledGoal schedule
                    ? schedule.getGoalPredicate(true).toString() : goal.getGoalPredicate().toString();
            records.add(new GoalRecoveryRecord(recoverySessionId, snapshot.gid, predicate, snapshot.status, claims));
          }
        }
        return records;
      }), cleanShutdown);
    } catch (IOException e) {
      recoveryFailure = e.toString();
      log.error("Could not checkpoint goal recovery; new execution is blocked", e);
      throw new UncheckedIOException(e);
    }
  }

  /**
   * Shutdown this ExecutionManager. All currently managed goals are canceled, ending the ActionInterpreter.
   * Note: the goals are automagically moved to the pastGoals list by the ActionListener mechanism.
   */
  public void shutdown() {
    CompletableFuture<Void> completion = beginShutdown();
    boolean interrupted = awaitShutdownCompletions(List.of(completion));
    try {
      completion.join();
    } finally {
      if (interrupted) {
        Thread.currentThread().interrupt();
      }
    }
  }

  /** Controller API: timing out never cancels a Future or releases unacknowledged resource claims. */
  public GoalManagerShutdownReport shutdownWithTimeout(long gracefulMillis, long interruptionMillis) {
    if (gracefulMillis < 0 || interruptionMillis < 0) {
      throw new IllegalArgumentException("Shutdown timeouts must be nonnegative");
    }
    CompletableFuture<Void> completion = beginShutdown();
    GoalManagerShutdownReport.Outcome outcome;
    String failure = null;
    try {
      try {
        completion.get(gracefulMillis, TimeUnit.MILLISECONDS);
      } catch (TimeoutException gracefulTimeout) {
        shutdownEscalated = true;
        if (executor instanceof GoalManagerExecutor workers) {
          workers.requestInterruption();
        }
        operationThreads.keySet().forEach(Thread::interrupt);
        completion.get(interruptionMillis, TimeUnit.MILLISECONDS);
      }
      outcome = GoalManagerShutdownReport.Outcome.COMPLETE;
    } catch (TimeoutException e) {
      outcome = GoalManagerShutdownReport.Outcome.TIMED_OUT;
      log.warn("Shutdown timed out in phase {}; unconfirmed work retains its claims", shutdownPhase);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      outcome = GoalManagerShutdownReport.Outcome.INTERRUPTED;
      log.warn("Shutdown caller interrupted; background shutdown continues");
    } catch (ExecutionException e) {
      outcome = GoalManagerShutdownReport.Outcome.FAILED;
      failure = e.getCause().toString();
    }
    List<GoalInfo> unfinished = new ArrayList<>();
    if (outcome != GoalManagerShutdownReport.Outcome.COMPLETE) {
      try {
        unfinished = coordinator.call(() -> {
          Set<Goal> goals = new HashSet<>(continuationGoals.values());
          registeredGoals.values().stream().filter(this::hasUnfinishedWork).forEach(goals::add);
          return goals.stream().sorted(Comparator.comparingLong(Goal::getId)).map(Goal::getInfo).toList();
        });
      } catch (RejectedExecutionException e) {
        if (!completion.isDone()) {
          throw e;
        }
      }
    }
    return new GoalManagerShutdownReport(outcome, shutdownEscalated, failure, shutdownPhase, unfinished);
  }

  private CompletableFuture<Void> beginShutdown() {
    synchronized (shutdownLock) {
      if (shutdownCompletion == null) {
        coordinator.run(() -> {
          shuttingDown = true;
          policyTimer.shutdownNow();
        });
        shutdownCompletion = new CompletableFuture<>();
        Thread controller = new Thread(() -> {
          try {
            finishShutdown();
            shutdownPhase = "COMPLETE";
            shutdownCompletion.complete(null);
          } catch (RuntimeException | Error failure) {
            log.error("Goal manager shutdown failed in phase {}", shutdownPhase, failure);
            shutdownCompletion.completeExceptionally(failure);
          }
        }, "goal-manager-shutdown");
        controller.setDaemon(true);
        controller.start();
      }
      return shutdownCompletion;
    }
  }

  private void finishShutdown() {
    log.info("Shutting down and cancelling all goals...");

    boolean interrupted = Thread.interrupted();
    try {
      shutdownPhase = "CHECKPOINTING";
      writeScheduledGoalsToFile();
      shutdownPhase = "CANCELING";
      List<Goal> cancel = new ArrayList<>(registeredGoals.values());
      List<CompletableFuture<?>> cancellations = new ArrayList<>();
      for (Goal goal : cancel) {
        if (!goal.isScheduled() && !goal.getStatus().isTerminated()) {
          cancellations.add(CompletableFuture.runAsync(() -> cancelGoal(goal.getId()), executor));
        }
      }
      shutdownPhase = "DRAINING_ADMISSIONS";
      List<CompletableFuture<?>> admissions = coordinator.call(() -> {
        List<CompletableFuture<?>> inFlight = new ArrayList<>(submissionsInFlight.values());
        inFlight.addAll(scheduledContinuations.values());
        return inFlight;
      });
      interrupted |= awaitShutdownCompletions(admissions);
      for (Goal goal : registeredGoals.values()) {
        if (goal.isScheduled() && !goal.getStatus().isTerminated()) {
          cancellations.add(CompletableFuture.runAsync(() -> cancelGoal(goal.getId()), executor));
        }
      }
      interrupted |= awaitShutdownCompletions(cancellations);
      shutdownPhase = "DRAINING_EXECUTIONS";
      List<CompletableFuture<?>> completions = coordinator.call(() -> {
        List<CompletableFuture<?>> futures = new ArrayList<>();
        for (StartingGoal attempt : executionAttempts.values()) {
          futures.add(attempt.startupFinished);
          futures.add(attempt.completion);
        }
        return futures;
      });
      interrupted |= awaitShutdownCompletions(completions);

      goalStatusUpdateLock.lock();
      try {
        while (registeredGoals.values().stream().anyMatch(goal -> !goal.getStatus().isTerminated())) {
          try {
            goalStatusUpdateCondition.await();
          } catch (InterruptedException e) {
            interrupted = true;
          }
        }
      } finally {
        goalStatusUpdateLock.unlock();
      }
      shutdownPhase = "CLOSING_INFRASTRUCTURE";
      scheduledGoalManager.shutdown();
      memoryManager.shutdown();
      while (!memoryManager.isTerminated()) {
        try {
          memoryManager.awaitTermination(Long.MAX_VALUE, TimeUnit.NANOSECONDS);
        } catch (InterruptedException e) {
          interrupted = true;
        }
      }
      sm.shutdown();
      executor.shutdown();
      while (!executor.isTerminated()) {
        try {
          executor.awaitTermination(Long.MAX_VALUE, TimeUnit.NANOSECONDS);
        } catch (InterruptedException e) {
          interrupted = true;
        }
      }
      synchronized (operationMonitor) {
        while (!operationThreads.isEmpty()) {
          try {
            operationMonitor.wait();
          } catch (InterruptedException e) {
            interrupted = true;
          }
        }
      }
      checkpointRecovery(true);
      if (recoveryStore != null) {
        try {
          recoveryStore.close();
        } catch (IOException e) {
          throw new UncheckedIOException(e);
        }
      }
      coordinator.close();
    } finally {
      if (interrupted) {
        Thread.currentThread().interrupt();
      }
    }
  }

  private boolean awaitShutdownCompletions(List<CompletableFuture<?>> completions) {
    boolean interrupted = false;
    for (CompletableFuture<?> completion : completions) {
      boolean finished = false;
      while (!finished) {
        try {
          completion.get();
          finished = true;
        } catch (InterruptedException e) {
          interrupted = true;
        } catch (ExecutionException e) {
          log.error("Goal execution failed while shutting down", e.getCause());
          finished = true;
        } catch (CancellationException e) {
          finished = true;
        }
      }
    }
    return interrupted;
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
    StartingGoal starting = coordinator.call(() -> {
      if (!registerGoal(g)) {
        throw new IllegalStateException("Context goal was already submitted: " + g.getId());
      }
      if (g.getStatus().isTerminated()) {
        transferGoalToPastGoals(g);
        return null;
      }
      StartingGoal reservation = new StartingGoal();
      startingGoals.put(g, reservation);
      executionAttempts.put(g, reservation);
      addActiveGoal(g, reservation.completion);
      return reservation;
    });
    if (starting == null) {
      throw new IllegalStateException("Cannot execute a terminal context goal: " + g.getId());
    }
    try {
      beginManagerOperation();
      checkpointRecovery(false);
      ActionInterpreter ai = ActionInterpreter.createInterpreterFromExecutionTree(g, context);
      Context simulationStartStep = ai.getCurrentStep();
      log.debug("Starting step " + simulationStartStep.getSignatureInPredicateForm());
      for (ActionListener al : aiListeners) {
        ai.addListener(al);
      }
      startActionInterpreter(ai);
      return simulationStartStep;
    } catch (RuntimeException | Error e) {
      log.error("Could not start execution tree for goal {}", g, e);
      g.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
              Factory.createPredicate("executionStartupFailed", g.getGoalPredicate())));
      throw e;
    } finally {
      try {
        finishStarting(g, starting);
      } finally {
        try {
          starting.startupFinished.complete(null);
          trimGoalHistory();
        } finally {
          endManagerOperation();
        }
      }
    }
  }

  /**
   * Configure defaults for newly constructed submissions. Direct Goal submissions retain their metadata.
   */
  public void configureGoalPriorities(Function<Predicate, PriorityTier> tierProvider,
                                      ToLongFunction<Predicate> priorityProvider) {
    priorityTierForGoal = Objects.requireNonNull(tierProvider);
    priorityForGoal = Objects.requireNonNull(priorityProvider);
  }

  public Goal submitGoal(Predicate predicate, ExecutionType type, PriorityTier tier,
                         Long priority, Symbol submitter, Predicate metric) {
    Goal goal = Goal.createGoal(predicate);
    prepareGoal(goal, predicate, type, tier, priority, submitter, metric);
    return submitGoal(goal);
  }

  public Goal submitGoalWithDeadline(Predicate predicate, long epochMillis) {
    Goal goal = Goal.createGoal(predicate);
    prepareGoal(goal, predicate, ExecutionType.ACT, null, null, Factory.createSymbol("unknown"), null);
    goal.setStartDeadline(epochMillis);
    return submitGoal(goal);
  }

  /** One priority point per interval of pending time, within the original tier. Zero disables aging. */
  public void configurePriorityAging(long intervalMillis) {
    if (intervalMillis < 0) {
      throw new IllegalArgumentException("Aging interval must be nonnegative");
    }
    coordinator.run(() -> {
      if (shuttingDown) {
        throw new RejectedExecutionException("Goal manager is shutting down");
      }
      if (agingTimer != null) {
        agingTimer.cancel(false);
        agingTimer = null;
      }
      agingIntervalMillis = intervalMillis;
      synchronized (pendingGoalsLock) {
        List<PendingGoal> entries = new ArrayList<>(pendingGoals);
        pendingGoals.clear();
        agedPriorities.clear();
        pendingSince.replaceAll((goal, since) -> schedulingTimeNanos());
        pendingGoals.addAll(entries);
      }
      if (intervalMillis > 0) {
        agingTimer = policyTimer.scheduleWithFixedDelay(
                () -> runPolicyTask(this::refreshSchedulingPolicies), 100, 100, TimeUnit.MILLISECONDS);
      }
      refreshPendingOrder();
    });
  }

  protected long schedulingTimeMillis() {
    return System.currentTimeMillis();
  }

  protected long schedulingTimeNanos() {
    return System.nanoTime();
  }

  private long effectivePriority(Goal goal) {
    return agedPriorities.getOrDefault(goal, goal.getPriority());
  }

  private void refreshPendingOrder() {
    if (agingIntervalMillis == 0) {
      return;
    }
    synchronized (pendingGoalsLock) {
      List<PendingGoal> entries = new ArrayList<>(pendingGoals);
      pendingGoals.clear();
      long now = schedulingTimeNanos();
      for (PendingGoal entry : entries) {
        Goal goal = entry.getGoal();
        long elapsed = TimeUnit.NANOSECONDS.toMillis(Math.max(0, now - pendingSince.getOrDefault(goal, now)));
        long credit = elapsed / agingIntervalMillis;
        long priority = goal.getPriority();
        agedPriorities.put(goal, priority > Long.MAX_VALUE - credit ? Long.MAX_VALUE : priority + credit);
      }
      pendingGoals.addAll(entries);
    }
  }

  /** Timer and decision-boundary entry point; aging alone does not interrupt running work. */
  protected void refreshSchedulingPolicies() {
    coordinator.run(() -> {
      if (shuttingDown) {
        return;
      }
      for (Goal goal : new ArrayList<>(deadlineTimers.keySet())) {
        expireUnstartedGoal(goal);
      }
      activateValidPendingGoals();
    });
  }

  private boolean expireUnstartedGoal(Goal goal) {
    GoalInfo info = goal.getInfo();
    StartingGoal attempt = executionAttempts.get(goal);
    if (info.status.isTerminated() || cancelingGoals.contains(goal)) {
      return true;
    }
    if (info.startDeadline == 0 || info.start >= 0 || (attempt != null && attempt.dispatched)
            || schedulingTimeMillis() < info.startDeadline) {
      return false;
    }
    cancelingGoals.add(goal);
    cancelDeadlineTimer(goal);
    coordinator.defer(() -> goal.setFailureStatus(GoalStatus.FAILED,
            new ConditionJustification(false, Factory.createPredicate("startDeadlineMissed", goal.getGoalPredicate()))));
    return true;
  }

  private void cancelDeadlineTimer(Goal goal) {
    ScheduledFuture<?> timer = deadlineTimers.remove(goal);
    if (timer != null) {
      timer.cancel(false);
    }
  }

  private void armDeadlineTimer(Goal goal) {
    deadlineTimers.put(goal, policyTimer.schedule(() -> runPolicyTask(() -> coordinator.run(() -> {
      if (!shuttingDown && !expireUnstartedGoal(goal) && goal.getInfo().start < 0
              && deadlineTimers.containsKey(goal)) {
        // A wall-clock correction can make a previously scheduled deadline wake too early.
        armDeadlineTimer(goal);
      } else {
        cancelDeadlineTimer(goal);
      }
    })), Math.max(0, goal.getInfo().startDeadline - schedulingTimeMillis()), TimeUnit.MILLISECONDS));
  }

  private void runPolicyTask(Runnable task) {
    try {
      executor.execute(() -> {
        if (policyTimer.isShutdown()) {
          return;
        }
        try {
          task.run();
        } catch (RejectedExecutionException e) {
          if (!policyTimer.isShutdown()) {
            log.error("Goal scheduling policy task was rejected", e);
          }
        } catch (RuntimeException e) {
          log.error("Goal scheduling policy task failed", e);
        }
      });
    } catch (RejectedExecutionException e) {
      if (!policyTimer.isShutdown()) {
        log.error("Goal scheduling policy task was rejected", e);
      }
    }
  }

  private void prepareGoal(Goal goal, Predicate predicate, ExecutionType type, PriorityTier tier,
                           Long priority, Symbol submitter, Predicate metric) {
    PriorityTier resolvedTier = Objects.requireNonNull(tier == null || tier == PriorityTier.UNINITIALIZED
            ? priorityTierForGoal.apply(predicate) : tier);
    long resolvedPriority = priority == null ? priorityForGoal.applyAsLong(predicate) : priority;
    goal.setExecutionType(Objects.requireNonNull(type));
    goal.setPriorityTier(resolvedTier);
    goal.setPriority(resolvedPriority);
    goal.setSubmitter(Objects.requireNonNull(submitter));
    goal.setMetric(metric);
  }

  void submitScheduledOccurrence(ScheduledGoal schedule, Goal occurrence) {
    beginManagerOperation();
    try {
      prepareAndSubmitScheduledOccurrence(schedule, occurrence);
    } catch (RejectedExecutionException e) {
      log.warn("Scheduled occurrence {} rejected by admission policy", occurrence.getId(), e);
      occurrence.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
              Factory.createPredicate("goalAdmissionRejected", occurrence.getGoalPredicate())));
    } finally {
      endManagerOperation();
    }
  }

  private void prepareAndSubmitScheduledOccurrence(ScheduledGoal schedule, Goal occurrence) {
    if (coordinator.call(() -> scheduledAdmissionClosed(schedule))) {
      occurrence.setStatus(GoalStatus.CANCELED);
      return;
    }
    try {
      prepareGoal(occurrence, occurrence.getGoalStatePredicate(), schedule.getExecutionType(),
              null, null, schedule.getSubmitter(), schedule.getMetric());
    } catch (RuntimeException e) {
      log.error("Could not prepare scheduled occurrence {}", occurrence, e);
      CompletableFuture<Void> submission = new CompletableFuture<>();
      boolean admitted = coordinator.call(() -> {
        if (scheduledAdmissionClosed(schedule)) {
          return false;
        }
        registerGoal(occurrence);
        submissionsInFlight.put(occurrence, submission);
        return true;
      });
      if (!admitted) {
        occurrence.setStatus(GoalStatus.CANCELED);
        return;
      }
      try {
        occurrence.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
                Factory.createPredicate("submissionPreparationFailed", occurrence.getGoalPredicate())));
      } finally {
        try {
          coordinator.run(() -> submissionsInFlight.remove(occurrence, submission));
        } finally {
          submission.complete(null);
        }
      }
      throw e;
    }
    if (schedule.isPeriodic() && occurrence.getPriorityTier() != PriorityTier.SKIPPENDING) {
      occurrence.setPriorityTier(PriorityTier.LOW);
    }
    occurrence.setScheduledState(2);
    submitGoal(occurrence, schedule);
  }

  private boolean scheduledAdmissionClosed(ScheduledGoal schedule) {
    return shuttingDown || schedule.isRecurrenceClosed() || schedule.getStatus().isTerminated()
            || cancelingGoals.contains(schedule);
  }

  void runScheduledContinuation(Goal occurrence, Runnable submission) {
    CompletableFuture<Void> finished = new CompletableFuture<>();
    boolean accepted;
    try {
      accepted = coordinator.call(() -> {
        if (shuttingDown) {
          return false;
        }
        checkAdmissionCapacity(occurrence);
        continuationGoals.put(occurrence.getId(), occurrence);
        scheduledContinuations.put(occurrence.getId(), finished);
        return true;
      });
    } catch (RejectedExecutionException e) {
      log.warn("Coalesced occurrence {} rejected by admission policy", occurrence.getId(), e);
      occurrence.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
              Factory.createPredicate("goalAdmissionRejected", occurrence.getGoalPredicate())));
      return;
    }
    if (!accepted) {
      occurrence.setStatus(GoalStatus.CANCELED);
      return;
    }
    try {
      executor.execute(() -> {
        try {
          submission.run();
        } catch (RuntimeException e) {
          log.error("Coalesced occurrence submission failed for {}", occurrence, e);
        } finally {
          finishScheduledContinuation(occurrence, finished);
        }
      });
    } catch (RejectedExecutionException e) {
      try {
        occurrence.setStatus(GoalStatus.CANCELED);
        log.error("Coalesced occurrence submission was rejected for {}", occurrence, e);
      } finally {
        finishScheduledContinuation(occurrence, finished);
      }
    }
  }

  private void finishScheduledContinuation(Goal occurrence, CompletableFuture<Void> finished) {
    try {
      coordinator.run(() -> {
        scheduledContinuations.remove(occurrence.getId(), finished);
        continuationGoals.remove(occurrence.getId(), occurrence);
      });
    } finally {
      finished.complete(null);
      trimGoalHistory();
    }
  }

  private boolean awaitScheduledContinuation(long gid, Long millis) {
    CompletableFuture<Void> submitting = scheduledContinuations.get(gid);
    if (submitting == null) {
      return true;
    }
    try {
      if (millis == null) {
        submitting.get();
      } else {
        submitting.get(Math.max(0, millis), TimeUnit.MILLISECONDS);
      }
      return true;
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      log.error("Interrupted while waiting for coalesced occurrence admission: {}", gid, e);
      return false;
    } catch (ExecutionException e) {
      log.error("Coalesced occurrence admission failed: {}", gid, e);
      return true;
    } catch (TimeoutException e) {
      return false;
    }
  }
}
