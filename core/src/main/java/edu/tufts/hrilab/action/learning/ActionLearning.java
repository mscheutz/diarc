/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.asl.AslBlockStructure;
import edu.tufts.hrilab.action.asl.AslScopeValidator;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Comparator;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.nio.file.StandardCopyOption;

/**
 * Top level class in charge of maintaining action learning, including the current learning state
 * and other paused learning states (a paused learning state is different from a parent learning state),
 * ...
 */
public class ActionLearning {
  private enum SessionPhase {
    ACTIVE,
    CLOSING,
    PAUSED,
    CANCELED,
    CLOSED
  }

  private enum WorkerState {
    STOPPED,
    STARTING,
    RUNNING
  }

  private enum InstructionOutcome {
    SUCCEEDED,
    CANCELED,
    FAILED
  }

  private record QueuedLearningGoal(Goal goal, LearningState learningState, long sessionId) {}

  private static final class SessionInstructionStatus {
    long accepted;
    long succeeded;
    long canceled;
    long failed;
    final List<String> failureMessages = new ArrayList<>();
  }

  private static final Logger log = LoggerFactory.getLogger(ActionLearning.class);
  private ExecutionManager executionManager;
  private boolean executeAction = false;
  private Set<Predicate> ignoreActionSet = new HashSet<>();
  private Queue<QueuedLearningGoal> goalQueue = new LinkedBlockingQueue<>();
  private Lock goalQueueEmptyLock = new ReentrantLock();
  private Condition goalQueueNotEmpty = goalQueueEmptyLock.newCondition();
  private Thread goalQueueManager;
  private WorkerState workerState = WorkerState.STOPPED;
  private long currentProcessingSessionId = -1L;
  private long nextSessionId = 1L;
  private final Map<LearningState, Long> learningStateSessionIds = new IdentityHashMap<>();
  private final Map<Long, SessionPhase> sessionPhases = new HashMap<>();
  private final Map<Long, SessionInstructionStatus> sessionInstructionStatuses = new HashMap<>();
  private Goal currentGoal = null;
  private LearningState currentLearningState = null;
  /**
   * Current action learning status. Only set this field via setLearningStatus method.
   */
  private ActionLearningStatus learningStatus = ActionLearningStatus.NONE;
  private Map<String, LearningState> pausedLearningStates = new HashMap<>();
  private Condition learningStateUpdateCondition = goalQueueEmptyLock.newCondition();
  private Map<Predicate,Boolean> learningStateUpdateListenerInterrupted = new HashMap<>();
  private final AtomicBoolean consumerBusy = new AtomicBoolean(false);
  private volatile Runnable beforeFinalizationValidationHook;
  private volatile Runnable beforeQueuedGoalProcessingHook;
  private volatile Runnable beforeQueuedGoalCreateEventSpecHook;
  private ActionLearningGui learningGui;
  private final BehaviorTreeLearning behaviorTreeLearning = new BehaviorTreeLearning();

  /**
   * Explicit structured teaching is separate from the linear instruction queue:
   * ordinary learned steps are not interpreted as implicit BT controls.
   */
  public BehaviorTreeLearning getBehaviorTreeLearning() {
    return behaviorTreeLearning;
  }

  /**
   * Learning status lock.
   */
  private Lock statusLock = new ReentrantLock();
  /**
   * Status lock condition that is signalled when the status changes.
   */
  private Condition statusCondition = statusLock.newCondition();

  /**
   * @param em Execution Manager instance to run executeWhileLearning and simulation goals
   */
  public ActionLearning(ExecutionManager em, boolean displayGui) {
    executionManager = em;

    if (displayGui) {
      log.debug("creating gui for action learning");
      learningGui = new ActionLearningGui();
    }

    //TODO:EW: I don't like this mechanism to determine what is a taught step vs a current command.
    //  No flexibility, hard to maintain
    //set which actions/goals to ignore and pass through to be executed during learning
    //ignoreActionSet.add(Factory.createPredicate("sayText(?actor,?text)"));
    ignoreActionSet.add(Factory.createPredicate("handled(?actor,?utterance)"));
    ignoreActionSet.add(Factory.createPredicate("modifyAction(?actor,?action,?modification,?location)"));
    ignoreActionSet.add(Factory.createPredicate("learnAction(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("endActionLearning(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("endTaskLearning(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("pauseActionLearning(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("resumeActionLearning(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("cancelActionLearning(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("beginLearnedIf(?actor,?action,?guard)"));
    ignoreActionSet.add(Factory.createPredicate("beginLearnedElseIf(?actor,?action,?guard)"));
    ignoreActionSet.add(Factory.createPredicate("beginLearnedElse(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("endLearnedIf(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("beginLearnedWhile(?actor,?action,?guard)"));
    ignoreActionSet.add(Factory.createPredicate("endLearnedWhile(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("beginLearnedForeach(?actor,?action,?iterator,?collection)"));
    ignoreActionSet.add(Factory.createPredicate("endLearnedForeach(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("beginLearnedFor(?actor,?action,?loopVar,?initialValue,?comparisonOperator,?comparisonValue,?update)"));
    ignoreActionSet.add(Factory.createPredicate("endLearnedFor(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("declareLearnedLocal(?actor,?action,?variableName,?javaType,?semanticType,?initializer)"));
    ignoreActionSet.add(Factory.createPredicate("endActionLearning(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("pauseActionLearning(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("resumeActionLearning(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelActionLearning(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("changeLearningExecution(?actor,?status)"));
    ignoreActionSet.add(Factory.createPredicate("endLearningAssembleScript(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("freeze(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("endFreeze(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("beginBehaviorTreeLearning(?actor,?name,?kind,?bindings)"));
    ignoreActionSet.add(Factory.createPredicate("beginBehaviorTreeLearning(?actor,?name,?kind,?bindings,?attributes)"));
    ignoreActionSet.add(Factory.createPredicate("addBehaviorTreeNode(?actor,?name,?parent,?id,?kind,?attributes,?index)"));
    ignoreActionSet.add(Factory.createPredicate("addBehaviorTreeDefinition(?actor,?name,?treeId,?rootId,?kind,?attributes)"));
    ignoreActionSet.add(Factory.createPredicate("addBehaviorTreeSubtree(?actor,?name,?parent,?id,?treeId,?remaps,?index)"));
    ignoreActionSet.add(Factory.createPredicate("addBehaviorTreeControl(?actor,?name,?parent,?id,?kind,?index)"));
    ignoreActionSet.add(Factory.createPredicate("addBehaviorTreeLeaf(?actor,?name,?parent,?id,?binding,?ports,?index)"));
    ignoreActionSet.add(Factory.createPredicate("recordBehaviorTreeStep(?actor,?name,?parent,?id,?binding,?step,?index)"));
    ignoreActionSet.add(Factory.createPredicate("removeBehaviorTreeNode(?actor,?name,?id)"));
    ignoreActionSet.add(Factory.createPredicate("moveBehaviorTreeNode(?actor,?name,?id,?parent,?index)"));
    ignoreActionSet.add(Factory.createPredicate("endBehaviorTreeLearning(?actor,?name)"));
    ignoreActionSet.add(Factory.createPredicate("cancelBehaviorTreeLearning(?actor,?name)"));
    ignoreActionSet.add(Factory.createPredicate("exportBehaviorTree(?actor,?name,?xml,?bindings)"));
    ignoreActionSet.add(Factory.createPredicate("getLearnedBehaviorTreeXml(?actor,?name)"));

    ignoreActionSet.add(Factory.createPredicate("cancelSystemGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("suspendSystemGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("suspendSystemGoalsBlocking(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("resumeSystemGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelAllCurrentGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelAllPendingGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelAllActiveGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelPendingGoalByIndex(?actor)"));

    //spotPickAndPlace - Only here because of workaround implementation of itk actions
    ignoreActionSet.add(Factory.createPredicate("believeObjectAt(?actor,?object,?pose)"));
    ignoreActionSet.add(Factory.createPredicate("describeObjectPose(?actor,?object)"));
    ignoreActionSet.add(Factory.createPredicate("describeObjectPoses(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("describeObjectsAt(?actor,?pose)"));
    ignoreActionSet.add(Factory.createPredicate("describeNumberOfObjectsAt(?actor,?pose)"));
    ignoreActionSet.add(Factory.createPredicate("describeCurrentGoal(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("describePendingGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("describeCurrentPose(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("describeDetectableObjectNames(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("describeKnownPoseNames(?actor)"));
  }

  public void startQueuingGoals() {
    Thread workerToStart = null;
    goalQueueEmptyLock.lock();
    try {
      if (workerState == WorkerState.RUNNING || workerState == WorkerState.STARTING) {
        return;
      }
      goalQueueManager = new Thread(this::runGoalQueueConsumer, "action-learning-goal-queue");
      workerState = WorkerState.STARTING;
      workerToStart = goalQueueManager;
    } finally {
      goalQueueEmptyLock.unlock();
    }
    workerToStart.start();
  }

  private void runGoalQueueConsumer() {
    goalQueueEmptyLock.lock();
    try {
      workerState = WorkerState.RUNNING;
      learningStateUpdateCondition.signalAll();
    } finally {
      goalQueueEmptyLock.unlock();
    }

    try {
      while (true) {
        QueuedLearningGoal queued = null;
        goalQueueEmptyLock.lock();
        try {
          while (goalQueue.isEmpty()) {
            if (!shouldWorkerStayAliveLocked()) {
              return;
            }
            try {
              goalQueueNotEmpty.await();
            } catch (InterruptedException e) {
              Thread.currentThread().interrupt();
              return;
            }
          }
          queued = goalQueue.remove();
          currentProcessingSessionId = queued.sessionId();
          consumerBusy.set(true);
        } finally {
          goalQueueEmptyLock.unlock();
        }

        processQueuedLearningGoal(queued);
      }
    } finally {
      consumerBusy.set(false);
      goalQueueEmptyLock.lock();
      try {
        currentProcessingSessionId = -1L;
        if (Thread.currentThread() == goalQueueManager) {
          goalQueueManager = null;
          workerState = WorkerState.STOPPED;
        }
        learningStateUpdateCondition.signalAll();
      } finally {
        goalQueueEmptyLock.unlock();
      }
    }
  }

  private boolean shouldWorkerStayAliveLocked() {
    return learningStatus == ActionLearningStatus.ACTIVE || !goalQueue.isEmpty();
  }

  private void processQueuedLearningGoal(QueuedLearningGoal queued) {
    Goal goal = queued.goal();
    long sessionId = queued.sessionId();
    LearningState sessionLearningState = queued.learningState();
    try {
      Runnable processingHook = beforeQueuedGoalProcessingHook;
      if (processingHook != null) {
        processingHook.run();
      }
      goalQueueEmptyLock.lock();
      try {
        if (!canApplyToLearningStateLocked(sessionId, sessionLearningState)) {
          recordInstructionOutcomeLocked(sessionId, InstructionOutcome.CANCELED,
              "Instruction canceled while queued: " + goal.getGoalStatePredicate());
          return;
        }
      } finally {
        goalQueueEmptyLock.unlock();
      }
      if (goal.getStatus().isTerminated()) {
        goalQueueEmptyLock.lock();
        try {
          recordInstructionOutcomeLocked(sessionId, InstructionOutcome.FAILED,
              "Instruction terminated before processing: " + goal.getGoalStatePredicate());
        } finally {
          goalQueueEmptyLock.unlock();
        }
        return;
      }
      if (shouldExecute()) {
        ignoreActionSet.add(goal.getGoalStatePredicate());
        goal.setIsLearningGoal(true);
        Goal submitted = executionManager.submitGoal(goal);
        GoalStatus goalStatus = executionManager.joinOnGoal(submitted.getId());
        ignoreActionSet.remove(goal.getGoalStatePredicate());

        if (goalStatus == GoalStatus.SUCCEEDED) {
          goalQueueEmptyLock.lock();
          try {
            if (!canApplyToLearningStateLocked(sessionId, sessionLearningState)) {
              recordInstructionOutcomeLocked(sessionId, InstructionOutcome.CANCELED,
                  "Instruction canceled before recording execution result: " + goal.getGoalStatePredicate());
              return;
            }
          } finally {
            goalQueueEmptyLock.unlock();
          }
          Runnable beforeCreateEventSpecHook = beforeQueuedGoalCreateEventSpecHook;
          if (beforeCreateEventSpecHook != null) {
            beforeCreateEventSpecHook.run();
          }
          goalQueueEmptyLock.lock();
          try {
            if (!canApplyToLearningStateLocked(sessionId, sessionLearningState)) {
              recordInstructionOutcomeLocked(sessionId, InstructionOutcome.CANCELED,
                  "Instruction canceled before recording execution result: " + goal.getGoalStatePredicate());
              return;
            }
            sessionLearningState.createEventSpec(submitted);
            recordInstructionOutcomeLocked(sessionId, InstructionOutcome.SUCCEEDED, null);
          } finally {
            goalQueueEmptyLock.unlock();
          }
        } else {
          goalQueueEmptyLock.lock();
          try {
            recordInstructionOutcomeLocked(sessionId, InstructionOutcome.FAILED,
                "Execution failed for accepted instruction: " + goalStatus + " " + submitted.getGoalStatePredicate());
          } finally {
            goalQueueEmptyLock.unlock();
          }
          log.error("not adding goal to learned action because it failed: {}", submitted);
        }
      } else {
        goal.setStatus(GoalStatus.SUCCEEDED);
        Runnable beforeCreateEventSpecHook = beforeQueuedGoalCreateEventSpecHook;
        if (beforeCreateEventSpecHook != null) {
          beforeCreateEventSpecHook.run();
        }
        goalQueueEmptyLock.lock();
        try {
          if (!canApplyToLearningStateLocked(sessionId, sessionLearningState)) {
            recordInstructionOutcomeLocked(sessionId, InstructionOutcome.CANCELED,
                "Instruction canceled before recording: " + goal.getGoalStatePredicate());
            return;
          }
          sessionLearningState.createEventSpec(goal);
          recordInstructionOutcomeLocked(sessionId, InstructionOutcome.SUCCEEDED, null);
        } finally {
          goalQueueEmptyLock.unlock();
        }
      }
    } catch (Exception e) {
      goalQueueEmptyLock.lock();
      try {
        recordInstructionOutcomeLocked(sessionId, InstructionOutcome.FAILED,
            "Instruction processing error for " + goal.getGoalStatePredicate() + ": "
                + e.getClass().getSimpleName() + ": " + e.getMessage());
      } finally {
        goalQueueEmptyLock.unlock();
      }
      log.error("[runGoalQueueConsumer] error while processing queued goal {}", goal, e);
    } finally {
      consumerBusy.set(false);
      goalQueueEmptyLock.lock();
      try {
        currentProcessingSessionId = -1L;
        learningStateUpdateCondition.signalAll();
      } finally {
        goalQueueEmptyLock.unlock();
      }
    }
  }

  /**
   * Helper method to lock when setting the action learning status. All code that sets this field should use
   * this method instead of setting it directly.
   *
   * @param status
   */
  private void setLearningStatus(ActionLearningStatus status) {
    statusLock.lock();
    try {
      learningStatus = status;
      statusCondition.signalAll();
    } finally {
      statusLock.unlock();
    }
  }

  public void waitForActionLearningStart(Predicate newAction) {
    statusLock.lock();
    try {
      while (!(learningStatus == ActionLearningStatus.ACTIVE && currentLearningState != null && newAction.getName().equalsIgnoreCase(currentLearningState.getName()))) {
        try {
          statusCondition.await();
        } catch (InterruptedException e) {
          log.warn("[waitForActionLearningStart] Interrupted while waiting for learning to start.", e);
        }
      }
    } finally {
      statusLock.unlock();
    }
  }

  void setBeforeFinalizationValidationHookForTest(Runnable hook) {
    beforeFinalizationValidationHook = hook;
  }

  void setBeforeQueuedGoalProcessingHookForTest(Runnable hook) {
    beforeQueuedGoalProcessingHook = hook;
  }

  void setBeforeQueuedGoalCreateEventSpecHookForTest(Runnable hook) {
    beforeQueuedGoalCreateEventSpecHook = hook;
  }

  Thread getQueueWorkerForTest() {
    goalQueueEmptyLock.lock();
    try {
      return goalQueueManager;
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  boolean waitForQueueWorkerStoppedForTest(long timeoutMillis) {
    long deadline = System.currentTimeMillis() + timeoutMillis;
    goalQueueEmptyLock.lock();
    try {
      while ((workerState == WorkerState.RUNNING || workerState == WorkerState.STARTING)
          && System.currentTimeMillis() < deadline) {
        long remaining = Math.max(1, deadline - System.currentTimeMillis());
        try {
          learningStateUpdateCondition.await(Math.min(remaining, 200), TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
        }
      }
      return workerState == WorkerState.STOPPED && goalQueueManager == null;
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  boolean waitForQueueWorkerRunningForTest(long timeoutMillis) {
    long deadline = System.currentTimeMillis() + timeoutMillis;
    goalQueueEmptyLock.lock();
    try {
      while (workerState != WorkerState.RUNNING && System.currentTimeMillis() < deadline) {
        long remaining = Math.max(1, deadline - System.currentTimeMillis());
        try {
          learningStateUpdateCondition.await(Math.min(remaining, 200), TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
        }
      }
      return workerState == WorkerState.RUNNING && goalQueueManager != null && goalQueueManager.isAlive();
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  private long registerSessionLocked(LearningState learningState, SessionPhase phase) {
    Long known = learningStateSessionIds.get(learningState);
    if (known != null) {
      sessionPhases.put(known, phase);
      sessionInstructionStatuses.computeIfAbsent(known, ignored -> new SessionInstructionStatus());
      return known;
    }
    long id = nextSessionId++;
    learningStateSessionIds.put(learningState, id);
    sessionPhases.put(id, phase);
    sessionInstructionStatuses.put(id, new SessionInstructionStatus());
    return id;
  }

  private Long sessionIdOfLocked(LearningState learningState) {
    if (learningState == null) {
      return null;
    }
    return learningStateSessionIds.get(learningState);
  }

  private SessionPhase sessionPhaseOfLocked(LearningState learningState) {
    Long sessionId = sessionIdOfLocked(learningState);
    if (sessionId == null) {
      return null;
    }
    return sessionPhases.get(sessionId);
  }

  private boolean transitionSessionPhaseLocked(LearningState learningState, SessionPhase expected, SessionPhase target) {
    Long sessionId = sessionIdOfLocked(learningState);
    if (sessionId == null) {
      return false;
    }
    SessionPhase phase = sessionPhases.get(sessionId);
    if (phase != expected) {
      return false;
    }
    sessionPhases.put(sessionId, target);
    return true;
  }

  private boolean transitionSessionPhaseLocked(LearningState learningState, SessionPhase target, SessionPhase... expectedPhases) {
    for (SessionPhase expectedPhase : expectedPhases) {
      if (transitionSessionPhaseLocked(learningState, expectedPhase, target)) {
        return true;
      }
    }
    return false;
  }

  private SessionPhase sessionPhaseByIdLocked(long sessionId) {
    return sessionPhases.get(sessionId);
  }

  private SessionInstructionStatus instructionStatusLocked(long sessionId) {
    return sessionInstructionStatuses.computeIfAbsent(sessionId, ignored -> new SessionInstructionStatus());
  }

  private void recordInstructionAcceptedLocked(long sessionId) {
    instructionStatusLocked(sessionId).accepted++;
  }

  private void recordInstructionOutcomeLocked(long sessionId, InstructionOutcome outcome, String message) {
    SessionInstructionStatus status = instructionStatusLocked(sessionId);
    switch (outcome) {
      case SUCCEEDED -> status.succeeded++;
      case CANCELED -> status.canceled++;
      case FAILED -> {
        status.failed++;
        if (message != null && !message.isBlank()) {
          status.failureMessages.add(message);
        }
      }
    }
  }

  private boolean allAcceptedInstructionsSucceededLocked(long sessionId) {
    SessionInstructionStatus status = sessionInstructionStatuses.get(sessionId);
    if (status == null) {
      return true;
    }
    return status.accepted == status.succeeded && status.canceled == 0 && status.failed == 0;
  }

  private String firstInstructionFailureLocked(long sessionId) {
    SessionInstructionStatus status = sessionInstructionStatuses.get(sessionId);
    if (status == null || status.failureMessages.isEmpty()) {
      return "";
    }
    return status.failureMessages.get(0);
  }

  private boolean canApplyToLearningStateLocked(long sessionId, LearningState learningState) {
    Long mappedSessionId = sessionIdOfLocked(learningState);
    if (mappedSessionId == null || mappedSessionId != sessionId) {
      return false;
    }
    SessionPhase phase = sessionPhaseByIdLocked(mappedSessionId);
    return phase == SessionPhase.ACTIVE || phase == SessionPhase.CLOSING;
  }

  private void retireSessionLocked(LearningState learningState, SessionPhase terminalPhase) {
    Long sessionId = sessionIdOfLocked(learningState);
    if (sessionId != null) {
      learningStateSessionIds.remove(learningState);
      sessionPhases.put(sessionId, terminalPhase);
    }
  }

  private int purgeQueuedGoalsForSessionLocked(long sessionId) {
    int removed = 0;
    java.util.Iterator<QueuedLearningGoal> iterator = goalQueue.iterator();
    while (iterator.hasNext()) {
      QueuedLearningGoal queued = iterator.next();
      if (queued.sessionId() == sessionId) {
        iterator.remove();
        recordInstructionOutcomeLocked(sessionId, InstructionOutcome.CANCELED,
            "Instruction canceled before processing: " + queued.goal().getGoalStatePredicate());
        removed++;
      }
    }
    return removed;
  }

  private void restoreSessionAfterFinalizationFailureLocked(long sessionId, SessionPhase previousPhase) {
    SessionPhase currentPhase = sessionPhaseByIdLocked(sessionId);
    if (currentPhase == SessionPhase.CLOSING && previousPhase != null
        && previousPhase != SessionPhase.CANCELED && previousPhase != SessionPhase.CLOSED) {
      sessionPhases.put(sessionId, previousPhase);
    }
  }

  private boolean canReactivateToActive(SessionPhase phase) {
    return phase == SessionPhase.PAUSED || phase == SessionPhase.ACTIVE;
  }

  //Start learning new action
  public boolean learnAction(Predicate newAction) {
    log.debug("[learnAction] " + newAction);
    String actName = newAction.getName();
    boolean activated = false;
    goalQueueEmptyLock.lock();
    try {
      if (learningStatus == ActionLearningStatus.ACTIVE) {
        if (currentLearningState == null) {
          log.warn("[learnAction] active status has no current learning session; rejecting {}", newAction);
          return false;
        }
        if (!currentLearningState.getName().equalsIgnoreCase(actName)) {
          SessionPhase parentPhase = sessionPhaseOfLocked(currentLearningState);
          if (parentPhase != SessionPhase.ACTIVE) {
            log.warn("[learnAction] cannot begin nested session {} while parent session {} is in phase {}",
                newAction, currentLearningState.getName(), parentPhase);
            return false;
          }
          currentLearningState = new LearningState(newAction, currentLearningState);
          registerSessionLocked(currentLearningState, SessionPhase.ACTIVE);
          activated = true;
        } else {
          SessionPhase phase = sessionPhaseOfLocked(currentLearningState);
          if (!canReactivateToActive(phase)) {
            log.warn("[learnAction] cannot reactivate session in phase {} for {}", phase, newAction);
            return false;
          }
          activated = transitionSessionPhaseLocked(currentLearningState, SessionPhase.ACTIVE,
              SessionPhase.PAUSED, SessionPhase.ACTIVE);
        }
      } else {
        currentLearningState = new LearningState(newAction);
        registerSessionLocked(currentLearningState, SessionPhase.ACTIVE);
        activated = true;
      }
      if (!activated) {
        log.warn("[learnAction] failed to activate session for {}", newAction);
        return false;
      }
      learningStatus = ActionLearningStatus.ACTIVE;
      goalQueueNotEmpty.signalAll();
      learningStateUpdateCondition.signalAll();
    } finally {
      goalQueueEmptyLock.unlock();
    }
    // TODO: spawn observers based on situation and task relevant info
    if (learningGui != null) {
      learningGui.setLearningState(currentLearningState);
    }
    setLearningStatus(ActionLearningStatus.ACTIVE);
    startQueuingGoals();

    //to keep the learning goal active while learning, wait here until status is no longer active
    statusLock.lock();
    try {
      while (learningStatus == ActionLearningStatus.ACTIVE) {
        try {
          statusCondition.await();
        } catch (InterruptedException e) {
          log.warn("Interrupted while waiting for learning to finish.", e);
        }
      }
    } finally {
      statusLock.unlock();
    }

    return true;
  }

  private boolean awaitQueueDrained(long timeoutMillis) {
    if (Thread.currentThread() == goalQueueManager) {
      return goalQueue.isEmpty() && !consumerBusy.get();
    }
    long deadline = System.currentTimeMillis() + timeoutMillis;
    goalQueueEmptyLock.lock();
    try {
      while ((!goalQueue.isEmpty() || consumerBusy.get()) && System.currentTimeMillis() < deadline) {
        long remaining = Math.max(1, deadline - System.currentTimeMillis());
        try {
          learningStateUpdateCondition.await(Math.min(remaining, 200), TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
        }
      }
      return goalQueue.isEmpty() && !consumerBusy.get();
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  private boolean runAfterQueueDrain(long timeoutMillis, Runnable mutation) {
    if (Thread.currentThread() == goalQueueManager) {
      mutation.run();
      return true;
    }
    long deadline = System.currentTimeMillis() + timeoutMillis;
    goalQueueEmptyLock.lock();
    try {
      while ((!goalQueue.isEmpty() || consumerBusy.get()) && System.currentTimeMillis() < deadline) {
        long remaining = Math.max(1, deadline - System.currentTimeMillis());
        try {
          learningStateUpdateCondition.await(Math.min(remaining, 200), TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
        }
      }
      if (!goalQueue.isEmpty() || consumerBusy.get()) {
        return false;
      }
      mutation.run();
      return true;
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  private boolean runAfterQueueDrainForActiveSession(Predicate action, long timeoutMillis, Runnable mutation) {
    if (action == null) {
      return false;
    }
    long deadline = System.currentTimeMillis() + timeoutMillis;
    goalQueueEmptyLock.lock();
    try {
      while ((!goalQueue.isEmpty() || consumerBusy.get()) && System.currentTimeMillis() < deadline) {
        long remaining = Math.max(1, deadline - System.currentTimeMillis());
        try {
          learningStateUpdateCondition.await(Math.min(remaining, 200), TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
        }
      }
      if (!goalQueue.isEmpty() || consumerBusy.get()) {
        return false;
      }
      if (currentLearningState == null || !action.getName().equalsIgnoreCase(currentLearningState.getName())) {
        return false;
      }
      SessionPhase phase = sessionPhaseOfLocked(currentLearningState);
      if (phase != SessionPhase.ACTIVE) {
        return false;
      }
      mutation.run();
      learningStateUpdateCondition.signalAll();
      return true;
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  private boolean awaitSessionDrained(long sessionId, long timeoutMillis) {
    if (Thread.currentThread() == goalQueueManager) {
      return !queueContainsSessionLockedUnsafe(sessionId) && currentProcessingSessionId != sessionId;
    }
    long deadline = System.currentTimeMillis() + timeoutMillis;
    goalQueueEmptyLock.lock();
    try {
      while ((queueContainsSessionLockedUnsafe(sessionId) || currentProcessingSessionId == sessionId)
          && System.currentTimeMillis() < deadline) {
        long remaining = Math.max(1, deadline - System.currentTimeMillis());
        try {
          learningStateUpdateCondition.await(Math.min(remaining, 200), TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
        }
      }
      return !queueContainsSessionLockedUnsafe(sessionId) && currentProcessingSessionId != sessionId;
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  private boolean queueContainsSessionLockedUnsafe(long sessionId) {
    for (QueuedLearningGoal queued : goalQueue) {
      if (queued.sessionId() == sessionId) {
        return true;
      }
    }
    return false;
  }

  //Cleanly finish learning, if parent learning state then resume learning
  public boolean endActionLearning(Predicate newAction) {
    log.debug("[endActionLearning] " + newAction);
    if (newAction == null) {
      log.warn("[endActionLearning] supplied action is null");
      return false;
    }
    if (currentLearningState == null && !pausedLearningStates.containsKey(newAction.getName())) {
      log.warn("[endActionLearning] no active or paused learning state for {}", newAction);
      return false;
    }

    boolean currentState;
    LearningState learningState;
    // check to see if the predicate action is same as action being learned
    if (currentLearningState != null && newAction.getName().equalsIgnoreCase(currentLearningState.getName())) {
      learningState = currentLearningState;
      currentState = true;
    } else {
      learningState = pausedLearningStates.get(newAction.getName());
      currentState = false;
      if (learningState == null) {
        log.error("[endActionLearning] called for action with no learning state: {}", newAction);
        return false;
      }
    }

    long finalizedSessionId;
    SessionPhase previousPhase;
    goalQueueEmptyLock.lock();
    try {
      previousPhase = sessionPhaseOfLocked(learningState);
      if (!transitionSessionPhaseLocked(learningState, SessionPhase.CLOSING, SessionPhase.ACTIVE, SessionPhase.PAUSED)) {
        log.warn("[endActionLearning] session already closing or closed for {}", newAction);
        return false;
      }
      Long finalizedSessionIdMaybe = sessionIdOfLocked(learningState);
      if (finalizedSessionIdMaybe == null) {
        log.warn("[endActionLearning] no session id available for {}", newAction);
        return false;
      }
      finalizedSessionId = finalizedSessionIdMaybe;
      goalQueueNotEmpty.signalAll();
      learningStateUpdateCondition.signalAll();
    } finally {
      goalQueueEmptyLock.unlock();
    }
    if (!awaitSessionDrained(finalizedSessionId, 30000)) {
      goalQueueEmptyLock.lock();
      try {
        restoreSessionAfterFinalizationFailureLocked(finalizedSessionId, previousPhase);
      } finally {
        goalQueueEmptyLock.unlock();
      }
      log.error("[endActionLearning] timed out waiting for accepted instructions for session {} to drain", newAction);
      return false;
    }
    goalQueueEmptyLock.lock();
    try {
      if (sessionPhaseByIdLocked(finalizedSessionId) != SessionPhase.CLOSING) {
        log.warn("[endActionLearning] session {} no longer closing during finalization", newAction);
        return false;
      }
      if (!allAcceptedInstructionsSucceededLocked(finalizedSessionId)) {
        restoreSessionAfterFinalizationFailureLocked(finalizedSessionId, previousPhase);
        String failureMessage = firstInstructionFailureLocked(finalizedSessionId);
        log.error("[endActionLearning] cannot publish action {} because accepted instruction processing failed. {}",
            newAction, failureMessage);
        return false;
      }
    } finally {
      goalQueueEmptyLock.unlock();
    }

    log.debug("Generating new ActionDBEntry");
    if (learningState.hasOpenBlocks()) {
      goalQueueEmptyLock.lock();
      try {
        restoreSessionAfterFinalizationFailureLocked(finalizedSessionId, previousPhase);
      } finally {
        goalQueueEmptyLock.unlock();
      }
      log.error("[endActionLearning] cannot publish action with unclosed control blocks: {}", newAction);
      return false;
    }
    Runnable finalizationHook = beforeFinalizationValidationHook;
    if (finalizationHook != null) {
      finalizationHook.run();
    }
    PublicationCandidate candidate;
    try {
      candidate = buildValidatedCandidate(learningState, "[endActionLearning]");
    } catch (Exception e) {
      goalQueueEmptyLock.lock();
      try {
        restoreSessionAfterFinalizationFailureLocked(finalizedSessionId, previousPhase);
      } finally {
        goalQueueEmptyLock.unlock();
      }
      log.error("[endActionLearning] failed to validate learned action {}", newAction, e);
      return false;
    }
    if (!publishCandidate(candidate, null, "[endActionLearning]")) {
      goalQueueEmptyLock.lock();
      try {
        restoreSessionAfterFinalizationFailureLocked(finalizedSessionId, previousPhase);
      } finally {
        goalQueueEmptyLock.unlock();
      }
      return false;
    }
    LearningState previousLearningState = learningState.getPreviousLearningState();
    goalQueueEmptyLock.lock();
    try {
      retireSessionLocked(learningState, SessionPhase.CLOSED);
    } finally {
      goalQueueEmptyLock.unlock();
    }
    if (previousLearningState != null) {
      log.debug("restoring previous action: " + learningState.getPreviousLearningState().getName());
      if (currentState) {
        currentLearningState = previousLearningState;
        goalQueueEmptyLock.lock();
        try {
          transitionSessionPhaseLocked(currentLearningState, SessionPhase.ACTIVE, SessionPhase.PAUSED, SessionPhase.ACTIVE);
        } finally {
          goalQueueEmptyLock.unlock();
        }
        startQueuingGoals();
        if (learningGui != null) {
          learningGui.setLearningState(currentLearningState);
        }
      } else {
        pausedLearningStates.put(newAction.getName(), previousLearningState);
      }
    } else {
      log.debug("ending learning for action {}", newAction);
      if (currentState) {
        setLearningStatus(ActionLearningStatus.END);
        currentLearningState = null;
        goalQueueEmptyLock.lock();
        try {
          goalQueueNotEmpty.signalAll();
          learningStateUpdateCondition.signalAll();
        } finally {
          goalQueueEmptyLock.unlock();
        }
        executeAction = false;
        if (learningGui != null) {
          learningGui.clear();
        }
      } else {
        pausedLearningStates.remove(newAction.getName());
      }
    }
    return true;
  }

  /**
   * Cancel action learning.
   *
   * @param action action being learned that should be cancelled
   * @return
   */
  public boolean cancelActionLearning(Predicate action) {
    log.debug("[cancelActionLearning] " + action);

    if (action == null) {
      log.warn("[cancelActionLearning] supplied action is null");
      return false;
    }
    LearningState activeAfterCancel = null;
    boolean canceled = false;
    boolean activeSessionCanceled = false;
    boolean endedAllLearning = false;
    goalQueueEmptyLock.lock();
    try {
      if (currentLearningState != null && action.getName().equalsIgnoreCase(currentLearningState.getName())) {
        SessionPhase currentPhase = sessionPhaseOfLocked(currentLearningState);
        if (currentPhase != SessionPhase.ACTIVE) {
          log.warn("[cancelActionLearning] cannot cancel active session {} in phase {}", action, currentPhase);
          return false;
        }
        Long canceledSessionId = sessionIdOfLocked(currentLearningState);
        LearningState canceledLearningState = currentLearningState;
        LearningState parentLearningState = canceledLearningState.getPreviousLearningState();
        if (!transitionSessionPhaseLocked(canceledLearningState, SessionPhase.ACTIVE, SessionPhase.CANCELED)) {
          return false;
        }
        retireSessionLocked(canceledLearningState, SessionPhase.CANCELED);
        if (canceledSessionId != null) {
          purgeQueuedGoalsForSessionLocked(canceledSessionId);
        }
        if (parentLearningState != null) {
          SessionPhase parentPhase = sessionPhaseOfLocked(parentLearningState);
          if (parentPhase == SessionPhase.ACTIVE
              || transitionSessionPhaseLocked(parentLearningState, SessionPhase.PAUSED, SessionPhase.ACTIVE)) {
            currentLearningState = parentLearningState;
            activeAfterCancel = parentLearningState;
          } else {
            currentLearningState = null;
            endedAllLearning = true;
          }
        } else {
          currentLearningState = null;
          endedAllLearning = true;
        }
        learningStatus = activeAfterCancel == null ? ActionLearningStatus.NONE : ActionLearningStatus.ACTIVE;
        goalQueueNotEmpty.signalAll();
        learningStateUpdateCondition.signalAll();
        canceled = true;
        activeSessionCanceled = true;
      } else {
        LearningState learningState = pausedLearningStates.remove(action.getName());
        if (learningState == null) {
          log.error("[cancelActionLearning] called for action with no learning state: {}", action);
          return false;
        }
        SessionPhase phase = sessionPhaseOfLocked(learningState);
        if (!transitionSessionPhaseLocked(learningState, SessionPhase.PAUSED, SessionPhase.CANCELED)) {
          log.warn("[cancelActionLearning] cannot cancel paused session {} in phase {}", action, phase);
          pausedLearningStates.put(action.getName(), learningState);
          return false;
        }
        Long sessionId = sessionIdOfLocked(learningState);
        retireSessionLocked(learningState, SessionPhase.CANCELED);
        if (sessionId != null) {
          purgeQueuedGoalsForSessionLocked(sessionId);
        }
        learningStateUpdateCondition.signalAll();
        canceled = true;
      }
    } finally {
      goalQueueEmptyLock.unlock();
    }

    if (!canceled) {
      return false;
    }

    if (!activeSessionCanceled) {
      return true;
    }

    setLearningStatus(ActionLearningStatus.CANCEL);
    if (activeAfterCancel != null) {
      setLearningStatus(ActionLearningStatus.ACTIVE);
      if (learningGui != null) {
        learningGui.setLearningState(activeAfterCancel);
      }
      startQueuingGoals();
      return true;
    }
    if (endedAllLearning) {
      executeAction = false;
      setLearningStatus(ActionLearningStatus.NONE);
      if (learningGui != null) {
        learningGui.clear();
      }
      return true;
    }
    setLearningStatus(ActionLearningStatus.NONE);
    return true;
  }

  /**
   * Pauses learning of action -> stores in paused map -> restored in future.
   *
   * @param action action being learned
   * @return
   */
  public boolean pauseActionLearning(Predicate action) {
    log.debug("[pauseActionLearning] " + action);

    if (action == null) {
      log.warn("[pauseActionLearning] supplied action is null");
      return false;
    }

    if (currentLearningState == null) {
      log.warn("[pauseActionLearning] Can't pause. Current learning state is null.");
      return false;
    } else if (action.getName().equalsIgnoreCase(currentLearningState.getName())) {
      AtomicBoolean pausedTransitionApplied = new AtomicBoolean(false);
      if (!runAfterQueueDrain(30000, () -> {
        if (!transitionSessionPhaseLocked(currentLearningState, SessionPhase.ACTIVE, SessionPhase.PAUSED)) {
          return;
        }
        pausedLearningStates.put(action.getName(), currentLearningState);
        currentLearningState = null;
        setLearningStatus(ActionLearningStatus.NONE);
        goalQueueNotEmpty.signalAll();
        pausedTransitionApplied.set(true);
      }) || !pausedTransitionApplied.get()) {
        log.error("[pauseActionLearning] timed out waiting for pending taught goals");
        return false;
      }
    } else {
      log.error("[pauseActionLearning] provided action is not equal to action currently being learned");
      return false;
    }
    return true;
  }

  /**
   * Resume the paused action, pop from map.
   *
   * @param action action being learned that should resume
   * @return
   */
  public boolean resumeActionLearning(Predicate action) {
    log.debug("[resumeActionLearning] " + action);

    if (action == null) {
      log.warn("[resumeActionLearning] supplied action is null");
      return false;
    }

    LearningState resumedLearningState = pausedLearningStates.get(action.getName());
    if (resumedLearningState != null) {
      goalQueueEmptyLock.lock();
      try {
        SessionPhase resumedPhase = sessionPhaseOfLocked(resumedLearningState);
        if (resumedPhase != SessionPhase.PAUSED) {
          log.error("cannot resume {} because session phase is {}", action.getName(), resumedPhase);
          return false;
        }
        if (currentLearningState != null) {
          if (!transitionSessionPhaseLocked(currentLearningState, SessionPhase.ACTIVE, SessionPhase.PAUSED)) {
            log.error("cannot stash currently active session {} while resuming {} due to phase mismatch",
                currentLearningState.getName(), action.getName());
            return false;
          }
          pausedLearningStates.put(currentLearningState.getName(), currentLearningState);
        }
        currentLearningState = resumedLearningState;
        pausedLearningStates.remove(action.getName());
        if (!transitionSessionPhaseLocked(currentLearningState, SessionPhase.PAUSED, SessionPhase.ACTIVE)) {
          log.error("cannot resume {} because session is no longer paused", action.getName());
          pausedLearningStates.put(action.getName(), resumedLearningState);
          return false;
        }
        goalQueueNotEmpty.signalAll();
        learningStateUpdateCondition.signalAll();
      } finally {
        goalQueueEmptyLock.unlock();
      }
      setLearningStatus(ActionLearningStatus.ACTIVE);
      startQueuingGoals();
      // TODO: restart / create observers
    } else {
      //tmf: agent should convey that it hasn't started learning something.
      // TODO: create interpreter to run say text to respond accordingly
      log.error("cannot resume " + action.getName() + " because haven't started learning before");
      return false;
    }
    return true;
  }

  public boolean beginLearnedIf(Predicate action, Predicate guard) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.beginLearnedIf(guard))) {
        log.error("[beginLearnedIf] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[beginLearnedIf] failed for {}", action, e);
      return false;
    }
  }

  public boolean beginLearnedElseIf(Predicate action, Predicate guard) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.beginLearnedElseIf(guard))) {
        log.error("[beginLearnedElseIf] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[beginLearnedElseIf] failed for {}", action, e);
      return false;
    }
  }

  public boolean beginLearnedElse(Predicate action) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.beginLearnedElse())) {
        log.error("[beginLearnedElse] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[beginLearnedElse] failed for {}", action, e);
      return false;
    }
  }

  public boolean endLearnedIf(Predicate action) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.endLearnedIf())) {
        log.error("[endLearnedIf] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[endLearnedIf] failed for {}", action, e);
      return false;
    }
  }

  public boolean beginLearnedWhile(Predicate action, Predicate guard) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.beginLearnedWhile(guard))) {
        log.error("[beginLearnedWhile] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[beginLearnedWhile] failed for {}", action, e);
      return false;
    }
  }

  public boolean endLearnedWhile(Predicate action) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.endLearnedWhile())) {
        log.error("[endLearnedWhile] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[endLearnedWhile] failed for {}", action, e);
      return false;
    }
  }

  public boolean beginLearnedForeach(Predicate action, String iteratorVariable, String collectionVariable) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000,
          () -> currentLearningState.beginLearnedForeach(iteratorVariable, collectionVariable))) {
        log.error("[beginLearnedForeach] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[beginLearnedForeach] failed for {}", action, e);
      return false;
    }
  }

  public boolean endLearnedForeach(Predicate action) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.endLearnedForeach())) {
        log.error("[endLearnedForeach] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[endLearnedForeach] failed for {}", action, e);
      return false;
    }
  }

  public boolean beginLearnedFor(Predicate action, String loopVariable, String initialValue,
                                 String comparisonOperator, String comparisonValue, String update) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000,
          () -> currentLearningState.beginLearnedFor(loopVariable, initialValue, comparisonOperator, comparisonValue, update))) {
        log.error("[beginLearnedFor] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[beginLearnedFor] failed for {}", action, e);
      return false;
    }
  }

  public boolean endLearnedFor(Predicate action) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000, () -> currentLearningState.endLearnedFor())) {
        log.error("[endLearnedFor] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[endLearnedFor] failed for {}", action, e);
      return false;
    }
  }

  public boolean declareLearnedLocal(Predicate action, String variableName, String javaType,
                                     String semanticType, String initializer) {
    try {
      if (!runAfterQueueDrainForActiveSession(action, 30000,
          () -> currentLearningState.declareLearnedLocal(variableName, javaType, semanticType, initializer))) {
        log.error("[declareLearnedLocal] timed out waiting for pending taught goals");
        return false;
      }
      return true;
    } catch (Exception e) {
      log.error("[declareLearnedLocal] failed for {}", action, e);
      return false;
    }
  }

  public boolean waitForLearningStateUpdate(Predicate action) {
    boolean updated = false;
    learningStateUpdateListenerInterrupted.put(action,false);
    while (!updated) {
      goalQueueEmptyLock.lock();
      try {
        learningStateUpdateCondition.await();
        if (learningStateUpdateListenerInterrupted.get(action) == true) {
          break;
        }
        //TODO: Why are the comparison checks only using name throughout this file. This isn't fully safe is it?
        if (currentLearningState != null && currentLearningState.getName().equalsIgnoreCase(action.getName())) {
          updated = true;
        }
      } catch (Exception e) {
        log.error("[waitForLearningStateUpdate]", e);
      } finally {
        goalQueueEmptyLock.unlock();
      }
    }
    return updated;
  }

  public void interruptWaitForLearningStateUpdate(Predicate action) {
    if (learningStateUpdateListenerInterrupted.containsKey(action)) {
      learningStateUpdateListenerInterrupted.put(action, true);
      goalQueueEmptyLock.lock();
      learningStateUpdateCondition.signal();
      goalQueueEmptyLock.unlock();
    } else {
      log.error("[interruptWaitForLearningStateUpdate] don't have pending listener for predicate {}", action);
    }
  }

  public boolean shouldIgnore(Goal goal) {
    return ignoreActionSet.stream().anyMatch(a -> goal.getGoalStatePredicate().instanceOf(a));
  }

  /**
   * check to see if the goal should be added to the action being learned and if so add it.
   *
   * @param goal goal to add
   */

  public void addGoal(Goal goal) {
    goalQueueEmptyLock.lock();
    try {
      if (learningStatus != ActionLearningStatus.ACTIVE || currentLearningState == null) {
        log.warn("[addGoal] dropping goal because there is no active learning session: {}", goal);
        return;
      }
      SessionPhase phase = sessionPhaseOfLocked(currentLearningState);
      Long sessionId = sessionIdOfLocked(currentLearningState);
      if (sessionId == null || phase != SessionPhase.ACTIVE) {
        log.warn("[addGoal] dropping goal because active session is not accepting instructions: {}", goal);
        return;
      }
      goalQueue.add(new QueuedLearningGoal(goal, currentLearningState, sessionId));
      recordInstructionAcceptedLocked(sessionId);
      goalQueueNotEmpty.signal();
      learningStateUpdateCondition.signal();
    } catch (Exception e) {
      log.error("[addGoal] ", e);
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  public ActionLearningStatus getLearningStatus() {
    return learningStatus;
  }

  public void changeLearningExecution(Symbol status) {
    executeAction = status.getName().equals("execute");
  }

  public boolean shouldExecute() {
    return executeAction;
  }

  //TODO: Shouldn't learning states be indexed by something more detailed than just signature name?
  public Predicate getActiveLearningSignature() {
    if (currentLearningState == null) {
      log.warn("[getActiveLearningName] method called but current learning state is null");
      return null;
    } else {
      List<Symbol> args = currentLearningState.getRoleNames();
      args.add(0, Factory.createVariable("?actor"));
      return Factory.createPredicate(currentLearningState.getName(),args);
    }
  }

  //TODO: cancelTeachingByModification
  //  Remove new entry that was formed but canceled halfway through teaching template
//Brad: moved from ActionModification.java which no longer exists.

  /**
   modifyAction(actor,action, modification, location)
   @param action : predicate containing action and arguments:
   arguments are required for grounding of new action/goal onto learning state
   actionName(actor, arg0, arg1, ... argN)
   predicate containing new action predicate and old task predicate
   like(newAction(arg0, arg1 ... argN), existingAction(arg0, arg1 ... argN)
   @param modification : information regarding the modification to make
   : type(actionStep, actionStep)
   : type(actionStep)
   : type(type, type) //TODO:brad is this actually valid?
   : type: insert, delete, replace
   : insert(action_predicate)
   : delete(action_predicate)
   : replace(new_action_predicate, old_action_predicate)
   @param location: reference in action script
   : relation(actionStep, actionStep)
   : relation(actionStep)
   : relation: and/before/after/condition/effect/argument
   : before(action_predicate)
   : after(action_predicate)
   : and(relation,relation))
   : condition()
   : effect()
   : argument()

   Structured edit operation forms (must use location predicate 'none'):
   : insertInBlock(blockStartIndex, actionPredicate)
   : insertInBranch(ifBlockStartIndex, branchIndex, actionPredicate)
   : removeBlock(blockStartIndex)
   : replaceBlock(blockStartIndex, actionPredicate)
   : addBlockLocal(insertIndex, variableName, javaType, semanticType [, initializer])
     Legacy compatibility: addBlockLocal(insertIndex, variableName, javaType, semanticType, none, initializer)
   : renameBlockLocal(declarationIndex, newName)
   : removeAt(index)
   : replaceAt(index, actionPredicate)
   Structured edits are transactional; rejected edits do not mutate the action.
   */

  public void modifyAction(Predicate action, Predicate modification, Predicate location) {
    /*
      check the name of the action predicate, if it is some value then, the inner
      predicates are actions themselves and denote a different action will be created

      currently:
        if the first argument of the action predicate is a predicate, then
          the system is learning a new action based on a known action
    */
    LearningState learningState;
    ActionDBEntry existingEntry = null;
    ActionDBEntry knownEntry;
    List<Symbol> args = action.getArgs();
    if (action.getName().equals("like")) {
      action = (Predicate) args.get(0);
      Predicate knownAction = (Predicate) args.get(1);
      knownEntry = getDBEntry(knownAction);
    } else {
      knownEntry = getDBEntry(action);
      existingEntry = knownEntry;
    }

    //TODO:brad:revisit the usage of learningState here?
    // get/create learning state
    if (knownEntry == null) {
      // this is the case when modifying an action while learning the action
      learningState = this.currentLearningState;
      if (learningState == null) {
        throw new IllegalArgumentException("No active learning state or matching known action for: " + action);
      }
    } else {
      // modifying an action that already exists
      learningState = new LearningState(knownEntry, action);
    }

    // finally, modify the action
    modifyAction(modification, location, learningState);

    PublicationCandidate candidate = buildValidatedCandidate(learningState, "[modifyAction]");
    if (!publishCandidate(candidate, existingEntry, "[modifyAction]")) {
      throw new IllegalStateException("Failed to publish modified action candidate for " + action.getName());
    }
  }

  private void modifyAction(Predicate modification, Predicate location, LearningState ls) {
    String operation = modification.getName();
    StructuredEditOperation structured = StructuredEditOperation.fromOperationName(operation);

    if (structured != null) {
      if (!"none".equalsIgnoreCase(location.getName())) {
        throw new IllegalArgumentException("Structured operation " + operation + " requires location predicate 'none'.");
      }
      structured.validateArity(modification);
      switch (structured) {
        case INSERT_IN_BLOCK -> {
        ls.appendActionToBlockBody(parseIntArg(modification.get(0), "blockStartIndex"), (Predicate) modification.get(1));
        return;
        }
        case INSERT_IN_BRANCH -> {
        ls.appendActionToBranch(
            parseIntArg(modification.get(0), "ifBlockStartIndex"),
            parseIntArg(modification.get(1), "branchIndex"),
            (Predicate) modification.get(2));
        return;
        }
        case REMOVE_BLOCK -> {
        ls.removeCompleteBlock(parseIntArg(modification.get(0), "blockStartIndex"));
        return;
        }
        case REPLACE_BLOCK -> {
        ls.replaceCompleteBlockWithAction(parseIntArg(modification.get(0), "blockStartIndex"), (Predicate) modification.get(1));
        return;
        }
        case ADD_BLOCK_LOCAL -> {
        String semanticType = nullableArg(modification.get(3));
        String initializer = parseBlockLocalInitializer(modification);
        ls.addBlockLocalDeclaration(
            parseIntArg(modification.get(0), "insertIndex"),
            modification.get(1).toString(),
            modification.get(2).toString(),
            semanticType,
            initializer);
        return;
        }
        case RENAME_BLOCK_LOCAL -> {
        ls.renameBlockLocalDeclaration(parseIntArg(modification.get(0), "declarationIndex"), modification.get(1).toString());
        return;
        }
        case REMOVE_AT -> {
        ls.removeActionAt(parseIntArg(modification.get(0), "index"));
        return;
        }
        case REPLACE_AT -> {
        ls.replaceActionAt(parseIntArg(modification.get(0), "index"), (Predicate) modification.get(1));
        return;
        }
      }
    }

    switch (location.getName()) {
      case "condition": {
        //TODO: handle disjunction;
        if (operation.equals("insert")) {
          ls.addCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(0), ConditionType.fromString(location.get(0).toString())));
        } else if (operation.equals("delete")) {
          ls.removeCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(0), ConditionType.fromString(location.get(0).toString())));
        } else { //TODO:brad: should we explicitly handle replace here? and fail otherwise?
          ls.removeCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(1), ConditionType.fromString(location.get(0).toString())));
          ls.addCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(0), ConditionType.fromString(location.get(0).toString())));
        }
        break;
      }
      case "effect": {
        if (operation.equals("insert")) {
          // TODO: fix comparison, cannot just add because no equals in effect
          ls.addEffect(new Effect((Predicate) modification.get(0), EffectType.fromString(location.get(0).toString())));
        } else if (operation.equals("delete")) {
          ls.removeEffect(new Effect((Predicate) modification.get(0), EffectType.fromString(location.get(0).toString())));
        } else {
          ls.removeEffect(new Effect((Predicate) modification.get(1), EffectType.fromString(location.get(0).toString())));
          ls.addEffect(new Effect((Predicate) modification.get(0), EffectType.fromString(location.get(0).toString())));
        }
        break;
      }
      case "argument":
        //TODO
        if (operation.equals("insert")) {
          log.warn("[modifyAction] inserting arguments not supported");
        } else if (operation.equals("delete")) {
          log.warn("[modifyAction] deleting arguments not supported");
        }
        break;
      default:  // modify action steps
        int modLocation;
        switch (operation) {
          case "removeLocalVar":
            ls.removeEventSpecsForLocalVariable(modification.get(0));
            break;
          case "insert":  // add action to script
            if (location.getName().equals("none")) {
               ls.addEventSpec(ls.getEventSpecs().size(), (Predicate) modification.get(0));
              log.warn("[modifyAction] inserting step at end of action");
              return;
            }
            modLocation = ls.getIndexOfConstraint(location);
            if (modLocation >= 0) {
              // TODO: this probably should not be done by extracting predicate, but idk
              ls.addEventSpec(modLocation, (Predicate) modification.get(0));
            } else {
              log.warn("[modifyAction] no event spec for " + location);
              return;
            }
            break;
          case "remove":  // remove action from script
            if (location.getName().equals("none")) {
              modLocation = ls.getEventStepLocation((Predicate) modification.get(0));
            } else {
              modLocation = ls.getIndexGivenConstraints(location, (Predicate) modification.get(0));
            }
            if (modLocation >= 0) {
              ls.removeEventSpec(modLocation);
            } else {
              log.warn("[modifyAction] no event spec for " + modification + " with reference step " + location);
              return;
            }
            break;
          case "replace":  // replace action within script
            Predicate oldStep = (Predicate) (modification.get(1));
            if (location.getName().equals("none")) {
              modLocation = ls.getEventStepLocation(oldStep);
            } else {
              modLocation = ls.getIndexGivenConstraints(location, oldStep);
            }
            if (modLocation >= 0) {
              // TODO: this probably should not be done by extracting predicate, but idk
              String newLv = ls.addEventSpec(modLocation, (Predicate) modification.get(0));
              ls.removeEventSpec(modLocation + 1, newLv);

            } else {
              log.warn("[modifyAction] no event spec for " + modification + " with reference step " + modLocation);
              return;
            }
            break;
          default:
            log.error("invalid action modification operation: " + operation + " in " + modification + " at " + location);
            break;
        }
        break;
    }
  }

  private record PublicationCandidate(ActionDBEntry candidate, String serialized, ActionDBEntry reparsed) {}

  private PublicationCandidate buildValidatedCandidate(LearningState learningState, String logContext) {
    ActionDBEntry candidate = learningState.generateActionDBEntry(false);
    validateCandidateStructureAndScope(candidate, logContext);
    ActionScriptLanguageWriter writer = new ActionScriptLanguageWriter();
    String serialized = writer.writeAction(candidate);
    ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
    List<ActionDBEntry> parsedRoundTrip = parser.parseStrict("<validated:" + candidate.getName() + ">", List.of(), serialized);
    if (parsedRoundTrip == null || parsedRoundTrip.size() != 1) {
      throw new IllegalStateException(logContext + " round-trip parse validation failed for action " + candidate.getName());
    }
    ActionDBEntry reparsed = parsedRoundTrip.get(0);
    if (!semanticallyEquivalent(candidate, reparsed)) {
      throw new IllegalStateException(logContext + " round-trip semantic mismatch for action " + candidate.getName());
    }
    return new PublicationCandidate(candidate, serialized, reparsed);
  }

  private void validateCandidateStructureAndScope(ActionDBEntry candidate, String logContext) {
    List<EventSpec> specs = candidate.getEventSpecs();
    AslBlockStructure.validateControlFlow(specs);
    AslScopeValidator.validate(specs, actionLocalRoleNames(candidate), actionRoleTypes(candidate));
    if (candidate.getSignature(true) == null) {
      throw new IllegalStateException(logContext + " candidate action has no signature");
    }
  }

  private List<String> actionLocalRoleNames(ActionDBEntry entry) {
    return entry.getRoles().stream()
        .filter(ActionBinding::isLocal)
        .map(ActionBinding::getName)
        .toList();
  }

  private Map<String, Class<?>> actionRoleTypes(ActionDBEntry entry) {
    Map<String, Class<?>> types = new HashMap<>();
    entry.getRoles().forEach(role -> types.put(role.getName(), role.getJavaType()));
    return types;
  }

  private boolean publishCandidate(PublicationCandidate publicationCandidate, ActionDBEntry existingEntry, String logContext) {
    ActionDBEntry candidate = publicationCandidate.candidate();
    Database database = Database.getInstance();
    boolean existingRemoved = false;
    try {
      if (existingEntry != null) {
        database.removeActionDBEntry(existingEntry);
        existingRemoved = true;
      }
      database.addActionDBEntry(candidate);
      if (!persistCandidate(publicationCandidate, existingEntry)) {
        database.removeActionDBEntry(candidate);
        if (existingRemoved) {
          database.addActionDBEntry(existingEntry);
        }
        return false;
      }
      return true;
    } catch (Exception e) {
      try {
        database.removeActionDBEntry(candidate);
      } catch (Exception ignored) {
      }
      if (existingRemoved) {
        try {
          database.addActionDBEntry(existingEntry);
        } catch (Exception rollbackError) {
          log.error("{} failed to roll back previous action after publish failure", logContext, rollbackError);
        }
      }
      log.error("{} failed to publish action {}", logContext, candidate.getName(), e);
      return false;
    }
  }

  private boolean persistCandidate(PublicationCandidate publicationCandidate, ActionDBEntry existingEntry) {
    if (executionManager == null || executionManager.getLearnedActionsFile() == null) {
      return true;
    }

    Path outputPath = Path.of(executionManager.getLearnedActionsFile());
    try {
      List<ActionDBEntry> persistedEntries = new ArrayList<>();
      if (Files.exists(outputPath)) {
        String current = Files.readString(outputPath);
        if (!current.isBlank()) {
          persistedEntries.addAll(new ActionScriptLanguageParser()
              .parseStrict(outputPath.toString(), List.of(), current));
        }
      }

      if (existingEntry != null) {
        persistedEntries.removeIf(entry -> sameSignature(entry, existingEntry));
      }
      persistedEntries.removeIf(entry -> sameSignature(entry, publicationCandidate.candidate()));
      persistedEntries.add(publicationCandidate.reparsed());

      StringBuilder contents = new StringBuilder();
      ActionScriptLanguageWriter writer = new ActionScriptLanguageWriter();
      for (ActionDBEntry entry : persistedEntries) {
        contents.append(writer.writeAction(entry)).append(System.lineSeparator());
      }

      Path directory = outputPath.toAbsolutePath().getParent();
      if (directory == null) {
        directory = Path.of(".");
      }
      Path temp = Files.createTempFile(directory, "learned-actions-", ".asl.tmp");
      try {
        Files.writeString(temp, contents.toString(), StandardOpenOption.TRUNCATE_EXISTING);
        try {
          Files.move(temp, outputPath, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        } catch (Exception atomicMoveError) {
          Files.move(temp, outputPath, StandardCopyOption.REPLACE_EXISTING);
        }
      } finally {
        Files.deleteIfExists(temp);
      }
      return true;
    } catch (Exception e) {
      log.error("[persistCandidate] failed to persist action {}", publicationCandidate.candidate().getName(), e);
      return false;
    }
  }

  private boolean semanticallyEquivalent(ActionDBEntry expected, ActionDBEntry actual) {
    if (!Objects.equals(expected.getName(), actual.getName())) {
      return false;
    }
    if (!Objects.equals(expected.getSignature(true), actual.getSignature(true))) {
      return false;
    }
    if (!normalizedRoles(expected).equals(normalizedRoles(actual))) {
      return false;
    }
    if (!normalizedEventSpecs(expected).equals(normalizedEventSpecs(actual))) {
      return false;
    }
    if (!normalizedConditions(expected.getPreConditions()).equals(normalizedConditions(actual.getPreConditions()))) {
      return false;
    }
    if (!normalizedConditions(expected.getOverallConditions()).equals(normalizedConditions(actual.getOverallConditions()))) {
      return false;
    }
    if (!normalizedEffects(expected.getEffects()).equals(normalizedEffects(actual.getEffects()))) {
      return false;
    }
    return Objects.equals(expected.getDescription(), actual.getDescription());
  }

  private List<String> normalizedRoles(ActionDBEntry entry) {
    return entry.getRoles().stream()
        .map(role -> role.getName() + "|" + role.getJavaType().getTypeName() + "|"
            + role.getSemanticType() + "|" + role.isLocal() + "|"
            + role.isReturn() + "|" + role.isVarArg() + "|"
            + role.hasDefaultValue() + "|"
            + (role.getDefaultValue() == null ? "<null>" : role.getDefaultValue()))
        .toList();
  }

  private List<String> normalizedEventSpecs(ActionDBEntry entry) {
    return entry.getEventSpecs().stream()
        .map(spec -> spec.getType() + "|" + spec.getActor() + "|" + spec.getCommand()
            + "|" + spec.getInputArgs() + "|" + spec.getReturnArgs())
        .toList();
  }

  private List<String> normalizedConditions(List<edu.tufts.hrilab.action.Condition> conditions) {
    return conditions.stream()
        .map(condition -> {
          StringBuilder sb = new StringBuilder(condition.getType().name());
          condition.getPredicates().entrySet().stream()
              .map(entry -> entry.getKey() + "@" + entry.getValue())
              .sorted(Comparator.naturalOrder())
              .forEachOrdered(value -> sb.append("|").append(value));
          return sb.toString();
        })
        .sorted(Comparator.naturalOrder())
        .toList();
  }

  private List<String> normalizedEffects(List<Effect> effects) {
    return effects.stream()
        .map(effect -> effect.getType() + ":" + effect.getPredicate())
        .sorted(Comparator.naturalOrder())
        .toList();
  }

  private boolean sameSignature(ActionDBEntry left, ActionDBEntry right) {
    if (left == null || right == null) {
      return false;
    }
    return Objects.equals(left.getSignature(true), right.getSignature(true));
  }

  private static int parseIntArg(Symbol symbol, String label) {
    try {
      return Integer.parseInt(symbol.toString());
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException("Expected integer " + label + " but got: " + symbol, e);
    }
  }

  private static String parseBlockLocalInitializer(Predicate modification) {
    int argCount = modification.getArgs().size();
    if (argCount <= 4) {
      return null;
    }
    if (argCount == 5) {
      return nullableArg(modification.get(4));
    }

    String legacySlot = nullableArg(modification.get(4));
    if (legacySlot != null) {
      throw new IllegalArgumentException("addBlockLocal legacy placeholder must be null/none when six arguments are used.");
    }
    return nullableArg(modification.get(5));
  }

  private static String nullableArg(Symbol symbol) {
    String value = symbol.toString();
    if (value.equalsIgnoreCase("null") || value.equalsIgnoreCase("none") || value.isEmpty()) {
      return null;
    }
    return value;
  }

  private ActionDBEntry getDBEntry(Predicate task) {
    List<ActionDBEntry> possibleEntries = Database.getActionDB().getActionsBySignature(task);
    if (!possibleEntries.isEmpty()) {
      return possibleEntries.get(0);
    }

    return Database.getActionDB().getAction(task.getName());
  }

  enum ModType {
    INSERT,
    DELETE,
    REPLACE;

    static boolean isModType(String type) {
      ModType val = Utilities.strToEnum(ModType.class, type);
      return val != null;
    }

    static ModType fromString(String typeStr) {
      if (typeStr != null) {
        for (ModType t : ModType.values()) {
          if (typeStr.equalsIgnoreCase(t.name())) {
            return t;
          }
        }
      }
      log.error("Unknown modification type: " + typeStr);
      return null;
    }
  }

  private enum StructuredEditOperation {
    INSERT_IN_BLOCK("insertInBlock", 2, 2),
    INSERT_IN_BRANCH("insertInBranch", 3, 3),
    REMOVE_BLOCK("removeBlock", 1, 1),
    REPLACE_BLOCK("replaceBlock", 2, 2),
    ADD_BLOCK_LOCAL("addBlockLocal", 4, 6),
    RENAME_BLOCK_LOCAL("renameBlockLocal", 2, 2),
    REMOVE_AT("removeAt", 1, 1),
    REPLACE_AT("replaceAt", 2, 2);

    private final String operationName;
    private final int minArgs;
    private final int maxArgs;

    StructuredEditOperation(String operationName, int minArgs, int maxArgs) {
      this.operationName = operationName;
      this.minArgs = minArgs;
      this.maxArgs = maxArgs;
    }

    private static StructuredEditOperation fromOperationName(String name) {
      for (StructuredEditOperation op : values()) {
        if (op.operationName.equals(name)) {
          return op;
        }
      }
      return null;
    }

    private void validateArity(Predicate modification) {
      int actual = modification.getArgs().size();
      if (actual < minArgs || actual > maxArgs) {
        throw new IllegalArgumentException("Operation " + operationName + " requires " + minArgs
            + ".." + maxArgs + " argument(s), but got " + actual + ": " + modification);
      }
    }
  }

  enum RelType {
    before,
    after;

    static boolean isRelType(String relation) {
      RelType val = Utilities.strToEnum(RelType.class, relation);
      return val != null;
    }

    static RelType fromString(String typeStr) {
      if (typeStr != null) {
        for (RelType t : RelType.values()) {
          if (typeStr.equalsIgnoreCase(t.name())) {
            return t;
          }
        }
      }
      log.error("Unknown relation type: " + typeStr);
      return null;
    }
  }

}
