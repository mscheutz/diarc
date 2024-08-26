/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.ContextTreeModification;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.execution.util.ContextUtils;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.State;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PerformanceAssessment {
  private static long lastQueryTime = 0;
  private static GoalManagerComponent goalManager;
  private static StateMachine stateMachine;
  private static Lock lock = new ReentrantLock();
  private static boolean runParallel = true;
  private static int defaultSamples = 50;
  private static Predicate lastGoal = null;
  private static Predicate lastMod = null;
  private static Symbol lastProb = Factory.createSymbol("");
  private static Symbol lastDur = Factory.createSymbol("");
  private static Symbol lastFail = Factory.createSymbol("");

 private static final Logger log = LoggerFactory.getLogger(PerformanceAssessment.class);

  public static void setGoalManager(GoalManagerComponent gm, boolean shouldRunParallel) {
    goalManager = gm;
    runParallel = shouldRunParallel;
  }

  /**
   * estimate the expected performance measures for a goal given a possible assessment type
   * @param goalPred the goal to assess
   * @param temporal (before, during, after) denotes if the goal hasn't been submitted, the agent is working toward the goal,
   *                 or the agent has completed the goal
   * @param assessmentModification possible modification to make for assessment
   *                               if(none()) -- no modification
   *                               if(complete(goal/action)) -- agent completes goal/action
   *                               if(state(stateToHold)) -- specific state holds
   *                               if(modify(modType(...)) -- modify action sequence
   *                                  insert(goal/action)
   *                                  delete(goal/action)
   *                                  replace(original goal/action, new goal/action)
   * @param numSamples number of simulation samples for assessment
   */
  public static Triple<Symbol, Symbol, Symbol> estimatePerformanceMeasures(Predicate goalPred, Symbol temporal, Predicate assessmentModification, int numSamples) {
    log.debug("checking to see if recently assessed: " + goalPred + " " + temporal + " execution with assessment modification " + assessmentModification + " " + numSamples + " times ");
    if (!shouldAssess(goalPred, assessmentModification)) {
      log.debug("recently assessed execution");
      return Triple.of(lastProb, lastDur, lastFail);
    }
    log.info("assessing: " + goalPred + " " + temporal + " execution with assessment modification " + assessmentModification + " " + numSamples + " times ");
    retractPriorAssessment(goalPred, assessmentModification, goalManager.getStateMachine());

    Pair<Goal, StateMachine> simInfo = prepareSimulation(goalPred, temporal, assessmentModification);
    Goal simGoal = simInfo.getLeft();
    StateMachine startingStateMachine = simInfo.getRight();
    if (simGoal == null || startingStateMachine == null) {
      return null;
    }

    Map<Set<Predicate>, StateDistribution> stateDistributions = simulateStateDistribution(simGoal, startingStateMachine, numSamples);

    long t0 = System.currentTimeMillis();
    Triple<Symbol, Symbol, Symbol> assessmentResults = assessStateDistributions(goalPred, stateDistributions, assessmentModification);
    long tTotal = System.currentTimeMillis() - t0;
    log.info( "distribution assessment took " + tTotal);
    lastQueryTime = System.currentTimeMillis();
    return assessmentResults;
  }

  /**
   * determine if the robot should assess its performance for a goal and modification, based on the last time it has assessed the same goal
   * @param goal the goal the robot should assess
   * @param assessmentModification possible assessment modifications
   * @return whether the robot should assess the goal with the modification
   */
  private static boolean shouldAssess(Predicate goal, Predicate assessmentModification) {
    int threshold = 90000; // how long to wait between queries to reassess (milliseconds)
    //threshold = 6; // how long to wait between queries to reassess (milliseconds)
    boolean assessAction = false;
    // TODO: better way to store lastQuery for specific assessment
    if (lastGoal == null || !(goal.toString().equals(lastGoal.toString()) && assessmentModification.toString().equals(lastMod.toString())) || (System.currentTimeMillis() > (lastQueryTime + threshold))) {
      assessAction = true;
    }
    lastGoal = goal.clone();
    lastMod = assessmentModification.clone();
    return assessAction;
  }

  /**
   * retract the prior assessment from belief for goal with possible modification
   * @param goal the goal which the robot should assess
   * @param assessmentModification possible assessment modifications
   * @param stateMachine state machine containing a reference to the belief component
   */
  private static void retractPriorAssessment(Predicate goal, Predicate assessmentModification, StateMachine stateMachine) {
    // TODO: only retract performance measures of same task and counterfactual modification
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("probabilityOf", "X", "Y", "W", "Z").toString()));
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("probabilityOf", "X", "Y", "W", "Z", "B").toString()));
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("durationOf", "X", "Y", "W", "Z").toString()));
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("durationOf", "X", "Y", "W", "Z", "B").toString()));
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("propertyOf", "X", "Y", "W", "Z").toString()));
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("propertyOf", "X", "Y", "W", "Z", "B").toString()));
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("mostLikelyToFailOf", "X", "Y","Z", "W").toString()));
    stateMachine.retractBelief(Factory.createPredicate("knows", "A", Factory.createPredicate("mostLikelyToFailOf", "X", "Y","Z", "W", "B").toString()));
  }

  //
  // prepare for simulation of goal
  //

  /**
   * create a simulation goal with a plan and state machine to be used for simulating performance
   * @param goalPred goal the system will simulate / assess
   * @param temporal before, during, or after execution
   * @param assessmentModification modification to make for assessment (none, complete, state, modify)
   * @return a goal and statemachine used for simulating
   */
  private static Pair<Goal, StateMachine> prepareSimulation(Predicate goalPred, Symbol temporal, Predicate assessmentModification) {
    Goal goal, simGoal;
    Pair<Goal, Goal> goals;
    if (temporal.getName().equals("before")) {
      goals = generateGoalPlan(goalPred, null); // simulate goal to get plan
    } else {
      goals = getGoalPlan(goalPred, temporal); // get current / past goal
      if (goals == null) {
        return Pair.of(null,null);
      }
    }
    goal = goals.getLeft();
    simGoal = goals.getRight();

    StateMachine simStateMachine = simGoal.getRootContext().getStateMachine();
    Predicate assessmentMod = Factory.createPredicate(assessmentModification.getArgs().get(0).toString()); // remove outer if()
    AssessmentType assessmentType = AssessmentType.fromString(assessmentMod.getName()); // complete, state, modify, none
    List<Symbol> modificationInfo = assessmentMod.getArgs();
    switch (assessmentType) {
      case COMPLETE: // complete(step, referenceLocation) -- complete an action step / goal at referenceLocation
                     // currently assumes no reference location
        simStateMachine = completeStep(goal, simGoal, temporal, modificationInfo);
        break;
      case STATE: // state(stateToHold)
        Predicate stateToHold = Factory.createPredicate(modificationInfo.get(0).toString());
        simStateMachine = modifyState(goal, simGoal, stateToHold, temporal, simStateMachine);
        break;
      case MODIFY: // insert(step,refLocation), delete(step,refLocation), replace(oldStep,newStep,refLocation)
                   // currently assumes no reference location
        simStateMachine = modifyGoalPlan(goal, simGoal, temporal, simStateMachine, modificationInfo);
      case NONE:
        break;
      default:
        log.warn("can't assess performance for execution modification of type: " + assessmentType);
        return Pair.of(null,null);
    }
    return Pair.of(simGoal, simStateMachine);
  }

  /**
   * submit goal to be simulated which generates a plan, clone the goal and top context, then resets the cloned child nodes
   * @param goalPred goal state to achieve
   * @return goal and clone of goal containing goalContext and children (children are reset)
   *         clonedGoal starting state is set to the state prior to simulating the goal
   */
  private static Pair<Goal, Goal> generateGoalPlan(Predicate goalPred, StateMachine simStateMachine) {
    // simulate goal to get plan
    Goal goal = goalManager.getGoal(goalManager.submitGoal(goalPred, ExecutionType.SIMULATE_PERFORMANCE));
    goalManager.joinOnGoal(goal.getId());
    // clone simulated goal and root context node
    Goal clonedGoal = new Goal(goal.getPredicate().clone());
    //clonedGoal.setStatus(GoalStatus.PENDING);
    // tmf: does system need to create simulated root?

    Context goalRootContext = goal.getRootContext();
    // get the state before simulation of the goal -- use the first non-GoalContext node because GoalContext not in SM
    State simStartingState = goalRootContext.getStateMachine().findStartingState(goalRootContext.getChildContexts().get(0), true);
    // create simulation root with simulation state machine
    Context simRootContext = goalManager.getExecutionManager().getRootContext().createSimulationRoot(new StateMachine(simStartingState), ExecutionType.SIMULATE_PERFORMANCE);
    // clone the context
    Context clonedContext = goal.getRootContext().copy(simRootContext);
    // reset children of the cloned root context node
    clonedContext.resetContext();
    clonedGoal.setRootContext(clonedContext);
    return Pair.of(goal,clonedGoal);
  }

  /**
   * locate current / past goal, copy context tree
   * @param goalPred goal state to achieve
   * @param temporal during (current goals) or after (past goals)
   * @return null or pair with goal and cloned goal with reset context tree with modification (if applicable)
   */
  private static Pair<Goal, Goal> getGoalPlan(Predicate goalPred, Symbol temporal) {
    Goal goal = findGoal(goalPred, temporal);
    if (goal == null) {
      return null;
    }
    Goal simGoal = new Goal(goal.getPredicate().clone());

    // following assumes top most state in statemachine hasn't been updated since execution of the goal
    Context goalContext = goal.getRootContext();
    // get the current state used to generate new state machine for simulation
    State simStartingState = goalContext.getStateMachine().getCurrentState();
    RootContext simRootContext = goalManager.getExecutionManager().getRootContext().createSimulationRoot(new StateMachine(simStartingState), ExecutionType.SIMULATE_PERFORMANCE);
    Context simGoalContext = goal.getRootContext().copy(simRootContext);
    simGoal.setRootContext(simGoalContext);
    return Pair.of(goal, simGoal);
  }

  /**
   * finds the most recent non-simulated goal matching goalPredicate in either current or past goals
   * @param goalPred goal predicate to locate
   * @param temporal current goals or past goals
   * @return most recent non-simulated goal
   */
  private static Goal findGoal(Predicate goalPred, Symbol temporal) {
    // locate current / past goal
    Goal goal = null;
    List<Goal> goals;

    if (temporal.toString().equals("during")) {
      goals = goalManager.getCurrentGoals();
    } else {
      goals = goalManager.getPastGoals();
    }
    boolean goalFound = false;
    for (Goal tmpGoal : goals) { // convert following to stream?
      // matching predicate && not simulation
      if (tmpGoal.getPredicate().instanceOf(goalPred) && !tmpGoal.getRootContext().getExecType().isSimulation()) {
        if (goalFound) {
          if (goal.getId() < tmpGoal.getId()) { // current goals are in map, so order may not be guaranteed?
            goal = tmpGoal;
          }
        } else {
          goalFound = true;
          goal = tmpGoal;
        }
      }
    }
    return goal;
  }

  /**
   * get the states which entail the success conditions for the goal
   * @param goal simulated goal which should hold
   * @param distribution the simulated state distribution
   * @return Map of the state to number of times the state was achieved
   */
  private static Map<Set<Predicate>, Integer> getSuccessStates(Goal goal, Map<Set<Predicate>, StateDistribution> distribution) {
    Map<Set<Predicate>, Integer> successStates = new HashMap<>();
    for (Set<Predicate> state : distribution.keySet()) {
      Predicate goalState = goal.getPredicate();
      if (goal.isAction()) {
        goalState = Factory.createPredicate("succeeded", goal.getPredicate());
      }
      if (goalState.isNegated()) {
        if (!state.contains(Factory.createPredicate(goalState.getArgs().get(0).toString()))) {
          Set<Predicate> tmpState = new HashSet<>(state);
          tmpState.add(goalState);
          successStates.put(tmpState, distribution.get(state).getTimesAchieved());
        }
      } else {
        if (state.contains(goalState)) {
          successStates.put(state, distribution.get(state).getTimesAchieved());
        }
      }
    }
    if (successStates.isEmpty()) {
      log.warn("could not locate success state for " + goal.getPredicate());
    }
    return successStates;
  }

  /**
   * samples a success state from all possible success states
   * @param successStates map from set of predicates to number of times it occurred
   * @return a success state
   */
  private static Set<Predicate> sampleSuccessState(Map<Set<Predicate>, Integer> successStates) {
    if (successStates.size() == 1) { // only one state
      return successStates.keySet().stream().findFirst().get();
    }
    //int total = 0;
    //total = successStates.values().stream().mapToInt(i -> i).sum();
    //for (Map.Entry<Set<Predicate>, Integer> state : successStates.entrySet()) {
    //  total += state.getValue();
    //}

    // for the moment return highest probability
    int numTimes = 0;
    Set<Predicate> sampledState = new HashSet<>();
    for (Map.Entry<Set<Predicate>, Integer> state : successStates.entrySet()) {
      if (state.getValue() > numTimes) {
        numTimes = state.getValue();
        sampledState = state.getKey();
      }
    }
    return sampledState;
  }

  /**
   * simulate the goal, extract the states which contain the goalState, sample from the set of states, update state
   * @param goalState the goal state the agent should achieve
   * @param stateMachine the state machine used to simulate the goal
   * @return a new state machine with an updated state
   */
  private static StateMachine simulateGoalState(Predicate goalState, StateMachine stateMachine) {
    StateMachine tmpStateMachine = new StateMachine(stateMachine,false);
    Goal stepGoal = new Goal(goalState);
    Pair<Goal, Goal> goals = generateGoalPlan(goalState, stateMachine); // no goal has been submitted so need to generate a new goal and a plan
    // get success state from assessment
    Map<Set<Predicate>, StateDistribution> distribution = simulateStateDistribution(goals.getRight(), tmpStateMachine, defaultSamples);
    Map<Set<Predicate>,Integer> successStates = getSuccessStates(stepGoal, distribution);
    if (successStates.isEmpty()) {
      return null;
    }
    Set<Predicate> sampledState = sampleSuccessState(successStates);

    // update state
    State tmpState = State.createRoot(stateMachine.getCurrentState());
    tmpState.addFacts(sampledState);
    return new StateMachine(tmpState);
  }

  /**
   * Complete the step specified in modificationInfo in order to get a new state for remaining simulation
   * @param goal - original goal used to get the correct starting state
   * @param simGoal - goal to be simulated
   * @param temporal before, during, after execution -- used to determine which state to acquire
   *                 during - if step to complete is in progress, it will revert to state prior to execution and simulate from first step
   * @param modificationInfo - agent, goal/action to complete
   * @return new state machine with updated state from completed step
   */
  // TODO: handle completion of future step and not just next step
  private static StateMachine completeStep(Goal goal, Goal simGoal, Symbol temporal, List<Symbol> modificationInfo) {
    Predicate step = Factory.createPredicate(modificationInfo.get(0).toString());
    Predicate refLocation = null;
    Context simGoalContext = simGoal.getRootContext().getChildContexts().get(0);
    // locate step to simulate
    Context stepToComplete = ContextUtils.getMatchingContext(simGoalContext, step, null, refLocation);
    Context originalStep = ContextUtils.getMatchingContext(goal.getRootContext(), step, null, refLocation);
    if (stepToComplete == null) {
      return null;
    }
    Map<Set<Predicate>, StateDistribution> distribution = new HashMap<>();
    Goal stepGoal = new Goal(step);
    StateMachine simStatMachine;
    State startingState = originalStep.getStateMachine().getCurrentState();
    ActionStatus stepStatus = originalStep.getStatus();
    StateMachine originalStepStateMachine = originalStep.getStateMachine();
    String temporalStr = temporal.getName();
    if (temporalStr.equals("before") || temporalStr.equals("during")) {
      int stepIndex = simGoalContext.getChildContexts().indexOf(stepToComplete);
      int currentStepIndex = Math.max(0,simGoalContext.getChildContexts().getNextIndex() - 1);
      // if stepToComplete is the current (the step about to be completed)
      // simulate stepToComplete using the current state,
      if (stepIndex == 0 || currentStepIndex == stepIndex) {
        startingState = simGoalContext.getStateMachine().getCurrentState();
        //simulate step
        StateMachine tmpStateMachine = new StateMachine(startingState);
        stepGoal = new Goal(step);
        stepGoal.setRootContext(stepToComplete);
        // simulate goal to get state distribution
        distribution = simulateStateDistribution(stepGoal, tmpStateMachine, defaultSamples);
      } else {
        // if stepToComplete is future step
        // simulate all steps leading up to and including the stepToComplete
        List<Context> stepsToSimulate = new ArrayList<>();
        if (temporalStr.equals("before")) {
          startingState = stepToComplete.getStateMachine().getCurrentState();
        }
        StateMachine tmpStateMachine = new StateMachine(startingState);
        Goal tmpGoal;
        for (int i=currentStepIndex; i < stepIndex; i++) {
          //stepsToSimulate.add(simGoalContext.getChildContexts().get(i));
          Context stepContext = simGoalContext.getChildContexts().get(i);
          tmpGoal = new Goal(stepContext.getSignatureInPredicateForm());
          tmpGoal.setRootContext(stepContext);
          Map<Set<Predicate>, StateDistribution> tmpDistribution = simulateStateDistribution(tmpGoal, tmpStateMachine, defaultSamples);
          Map<Set<Predicate>,Integer> successStates = getSuccessStates(tmpGoal, tmpDistribution);
          if (successStates.isEmpty()) {
            return null;
          }
          Set<Predicate> sampledState = sampleSuccessState(successStates);

          // update state
          State tmpState = State.createRoot(startingState);
          tmpState.addFacts(sampledState);
          tmpStateMachine = new StateMachine(tmpState);
          startingState = tmpStateMachine.getCurrentState();
        }

        stepGoal = new Goal(step);
        stepGoal.setRootContext(stepToComplete);
        distribution = simulateStateDistribution(stepGoal, tmpStateMachine, defaultSamples);
      }
    } else {
      startingState = originalStepStateMachine.findStartingState(originalStep, true);
      // reset context
      stepToComplete.resetContext();

      StateMachine tmpStateMachine = new StateMachine(startingState);
      stepGoal.setRootContext(stepToComplete);
      // simulate goal to get state distribution
      distribution = simulateStateDistribution(stepGoal, tmpStateMachine, defaultSamples);
    }

    // get success state from assessment
    Map<Set<Predicate>,Integer> successStates = getSuccessStates(stepGoal, distribution);
    if (successStates.isEmpty()) {
      return null;
    }
    Set<Predicate> sampledState = sampleSuccessState(successStates);

    // update state
    State tmpState = State.createRoot(startingState);
    tmpState.addFacts(sampledState);
    ContextTreeModification.goToNextStep(simGoalContext, stepToComplete);

    return new StateMachine(tmpState);
  }

  /**
   * Modify the state of the world based on the modification
   * @param goal original goal -- used to get failure context and state
   * @param simGoal new goal for simulation -- if failed, then context tree is updated
   * @param stateToHold the condition which the robot should assume holds
   * @param temporal before, during, after execution -- used to determine which state to acquire
   *                 before assumes modify original starting state
   *                 during assumes current state
   *                 after state is dependent on result of original execution
   * @param simStateMachine the copied state machine which will be modified
   */
  private static StateMachine modifyState(Goal goal, Goal simGoal, Predicate stateToHold, Symbol temporal, StateMachine simStateMachine) {
    Context simGoalContext = simGoal.getRootContext();
    Context goalContext = goal.getRootContext();
    if (temporal.getName().equals("after")) { // goal failed, need to find correct starting state and update it based on the state that needs to hold
        // TODO: if failed post-condition -> revert back to state prior to execution, forward simulate that action again, set action success, move to next step
        //       if failed pre-condition -> maintain same state, perform action to achieve desired state, reset failed action

      Context failedContext = ContextUtils.getFailureContext(goalContext);
      if (failedContext == null) {
        log.warn("[modifyState] couldn't locate failed context even though context failed");
        return null;
      }
      ActionStatus failStatus = failedContext.getStatus();
      simStateMachine = goalContext.getStateMachine();
      State tmpState = simStateMachine.findStartingState(failedContext, true);
      simStateMachine = new StateMachine(tmpState);
      if (failStatus.equals(ActionStatus.FAIL_PRECONDITIONS)) {
        simStateMachine = simulateGoalState(stateToHold, simStateMachine);
        ContextTreeModification.resetFailedContext(simGoalContext);
      } else if (failStatus.equals(ActionStatus.FAIL_POSTCONDITIONS)) {
        // TODO: how should this be handled?
        //       should system simulate failed step and get correct resulting state
        //       or
        //       select an action to simulate which will result in the correct state
        RootContext tmpRoot = new RootContext(new ActionConstraints(), new StateMachine(tmpState), ExecutionType.SIMULATE_PERFORMANCE);
        Goal tmpGoal = new Goal(simGoal.getPredicate());
        Context tmpContext = failedContext.copy(tmpRoot);
        tmpGoal.setRootContext(tmpContext);

        goalManager.getExecutionManager().submitGoalContext(tmpGoal, tmpContext);
        goalManager.joinOnGoal(tmpGoal.getId());

      }
    } else {
      simStateMachine = simulateGoalState(stateToHold, simStateMachine);
    }
    return simStateMachine;
  }

  /**
   * modify the simulated goal plan for assessment
   * @param goal original goal -- used for getting the starting state
   * @param simGoal new goal for simulation -- updated based on modification info
   * @param temporal before, during, after execution
   *                 before -- modifying any step within top level context
   *                 during -- modifying any remaining step within top level context
   *                 after -- modifying any executed step within top level context
   * @param simStateMachine simulated state machine which may be updated if action sequence failed
   * @param modificationInfo modType(...) -- modify action sequence
   *                         insert(goal/action)
   *                         delete(goal/action)
   *                         replace(original goal/action, new goal/action)
   */
  //TODO: include reference location for modification
  private static StateMachine modifyGoalPlan(Goal goal, Goal simGoal, Symbol temporal, StateMachine simStateMachine, List<Symbol> modificationInfo) {
    Context simRootContext = simGoal.getRootContext();
    Context goalRootContext = goal.getRootContext();
    StateMachine goalStateMachine = goalRootContext.getStateMachine();
    State state = simRootContext.getStateMachine().getCurrentState();

    int modStepNum = ContextTreeModification.modifyContext(simRootContext, Factory.createPredicate(modificationInfo.get(0).toString()));
    if (temporal.getName().equals("during")) {
      // if the step at index prior to the modification location
      // if modified already executed step, need to revert back to state before modification
      if (modStepNum < goalRootContext.getChildContexts().getNextIndex()) {
        Context priorContext = goalRootContext.getChildContexts().get(modStepNum);
        state = goalStateMachine.findResultingState(priorContext);
      }
    }
    if (temporal.getName().equals("after")) {
      // TODO: handle modification of prior steps -- need to correctly set state to the state before the modification location
      if (modStepNum < goalRootContext.getChildContexts().getNextIndex()) {

      }

      if (goalRootContext.isFailure()) {
        state = getStateBeforeFailure(goal, simStateMachine);
        simStateMachine = new StateMachine(state);
      }
    }
    return new StateMachine(state);
  }

  /**
   * acquire a copy of the state before execution of the context that caused the failure
   * @param goal original goal -- used to get the correct state from the state machine
   * @param simStateMachine state machine -- used to get correct state
   * @return copy of the state before failure
   */
  // TODO: is it necessary to pass in the simulated state machine? why not use the state machine from the goal root context?
  private static State getStateBeforeFailure(Goal goal, StateMachine simStateMachine) {
    Context failedContext = ContextUtils.getFailureContext(goal.getRootContext());
    State state = simStateMachine.findStartingState(failedContext);
    return new State(state);
  }


  //
  // Simulate execution
  //

  /**
   * Simulate the goal to get a resulting state distribution
   * @param simGoal goal containing a plan / action trace to be simulated
   * @param initialSateMachine StateMachine with correct initial state
   * @param numSamples number of times to simulate
   * @return resulting state distribution from simulations
   */
  private static Map<Set<Predicate>, StateDistribution> simulateStateDistribution(Goal simGoal, StateMachine initialSateMachine, int numSamples) {
    HashMap<Set<Predicate>, StateDistribution> stateDistributions = new HashMap<>();
    ExecutorService executor = Executors.newCachedThreadPool();
    long tInit = System.currentTimeMillis();
    for (int index=0; index < numSamples; index++) {
      if (runParallel) {
        SimulateGoal simulateGoal = new SimulateGoal(simGoal, initialSateMachine, stateDistributions, index);
        executor.submit(simulateGoal);
      } else {
        Pair<Goal, Context> simInfo = executeSimulation(simGoal, initialSateMachine);
        SimulationResult simulationResult = processGoalSimulation(simInfo.getLeft(), simInfo.getRight());
        if (simulationResult != null) {
          updateStateDistributions(simulationResult, stateDistributions);
        }
      }
    }
    if (runParallel) {
      try {
        executor.shutdown();
        boolean results = executor.awaitTermination(Long.MAX_VALUE, TimeUnit.SECONDS);
        //boolean results = executor.awaitTermination(30, TimeUnit.SECONDS);
        if (!results) {
          log.warn("could not finish simulations");
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    long tTotal = System.currentTimeMillis() - tInit;
    log.info(numSamples + " simulations took " + tTotal);
    return stateDistributions;
  }

  /**
   * execute the singular simulation instance
   * @param goal goal to simulate
   * @param stateMachine simulated state machine
   * @return the starting context
   */
  private static Pair<Goal, Context> executeSimulation(Goal goal, StateMachine stateMachine) {
    long t0 = System.currentTimeMillis();
    // clone the original goal for simulation
    Goal simGoal = new Goal(goal.getPredicate().clone());
    // clone state machine from cloned goal's root context -- should point to the correct starting state
    Context goalRootContext = goal.getRootContext();
    StateMachine simStateMachine = new StateMachine(stateMachine, false);
    // generate new root context for cloned context tree
    RootContext simRootContext = new RootContext(new ActionConstraints(), simStateMachine, ExecutionType.SIMULATE_PERFORMANCE);
    Context simContext;
    // get the first non GoalContext -- assumes the goals 'rootContext' is actual a GoalContext
    // if system uses actual GoalContext, then it will select new plan to execute instead of the same plan
    if (goalRootContext instanceof GoalContext) {
      simContext = goal.getRootContext().getChildContexts().get(0).copy(simRootContext);
    } else {
      simContext = goal.getRootContext().copy(simRootContext);
    }
    Context simulationStartStep = goalManager.getExecutionManager().submitGoalContext(simGoal, simContext);
    goalManager.joinOnGoal(simGoal.getId());
    long dur1 = System.currentTimeMillis()-t0;
    // FIXME: need better way to get child if start step is a goal context
    if (simulationStartStep instanceof GoalContext) {
      simulationStartStep = simulationStartStep.getChildContexts().get(0);
    }
    long dur2 = System.currentTimeMillis()-t0;
    return Pair.of(simGoal, simulationStartStep);
  }

  //
  // process resulting execution & assess results
  //

  private static void updateStateDistributions(PerformanceAssessment.SimulationResult simulationResult, Map<Set<Predicate>, PerformanceAssessment.StateDistribution> stateDistributions) {
    try {
      lock.lock();
      if (!stateDistributions.containsKey(simulationResult.resultingState)) {
        stateDistributions.put(simulationResult.resultingState, new PerformanceAssessment.StateDistribution(simulationResult.duration, simulationResult.actionTrace, simulationResult.failureReasons));
      } else {
        PerformanceAssessment.StateDistribution distribution = stateDistributions.get(simulationResult.resultingState);
        distribution.updateDistribution(simulationResult.duration, simulationResult.actionTrace, simulationResult.failureReasons);
      }
    } finally {
      lock.unlock();
    }
  }

  /**
   * Use the resulting simulated state distribution to calculate the probability of success, expected time, and step most likely to fail
   * @param goal goal the agent needs to assess expected performance
   * @param stateDistribution resulting state distribution of resulting states to action traces
   * @param counterfacutal hypothetical or counterfactual assessment modifications
   **/
  private static Triple<Symbol, Symbol, Symbol> assessStateDistributions(Predicate goal, Map<Set<Predicate>, PerformanceAssessment.StateDistribution> stateDistribution, Predicate counterfacutal) {
    double totalExecutions = 0;
    int numSuccess = 0;
    Symbol agent = goal.getArgs().get(0);
    Pair<Double, Double> time = Pair.of(0.0,0.0);
    HashMap<Set<Predicate>, PerformanceAssessment.StateDistribution> successSimulations = new HashMap<>();
    PerformanceAssessment.StateDistribution mostLikelyFail = null;
    HashMap<Set<Predicate>, Integer> executionFailureReasons = new HashMap<>();
    Set<Predicate> highestFailureReasons = null;
    HashMap<Predicate, Integer> mostLikelyStepFailures = new HashMap<>();
    Predicate mostLikelyStepFailure = null;

    for (Map.Entry<Set<Predicate>, PerformanceAssessment.StateDistribution> termState : stateDistribution.entrySet()) {
      PerformanceAssessment.StateDistribution dist = termState.getValue();
      Set<Predicate> state = termState.getKey();
      totalExecutions += dist.getTimesAchieved();

      Predicate stateToFind = goal;
      // check to see if goal refers to an action - wrap goal with 'succeeded'
      if (!Database.getActionDB().getActionsBySignature(goal).isEmpty()) {
        stateToFind = Factory.createPredicate("succeeded", goal);
      }
      if ((stateToFind.isNegated() && !state.contains(Factory.createNegatedPredicate(stateToFind))) || state.contains(stateToFind)) {
        time = dist.getTimeInfo();
        successSimulations.put(state, dist);
        numSuccess += dist.getTimesAchieved();
      } else {
        for (PerformanceAssessment.ActionTrace trace : dist.getActionTraces()) {
          int val = 1;
          Predicate fStep = trace.getFailureStep();
          if (fStep != null) {
            if (mostLikelyStepFailures.containsKey(fStep)) {
              val += mostLikelyStepFailures.get(fStep);
              if (mostLikelyStepFailure == null || mostLikelyStepFailures.get(mostLikelyStepFailure) < val) {
                mostLikelyStepFailure = fStep;
              }
            }
            mostLikelyStepFailures.put(fStep,val);
            if (mostLikelyStepFailure == null) {
              mostLikelyStepFailure = fStep;
            }
          }
        }
        for (Set<Predicate> fReasons : dist.getFailureReasons()) {
          int val = 1;
          if (executionFailureReasons.containsKey(fReasons)) {
            val += executionFailureReasons.get(fReasons);
            if (highestFailureReasons == null || executionFailureReasons.get(highestFailureReasons) < val){
              highestFailureReasons = fReasons;
            }
          }
          executionFailureReasons.put(fReasons,val);
        }
        if (mostLikelyFail == null || mostLikelyFail.getTimesAchieved() < dist.getTimesAchieved()) {
          mostLikelyFail = dist;
        }
      }
    }
    if (totalExecutions == 0) {
      log.warn("simulated the goal, but got 0 executions... something is wrong");
      return null;
    }

    Predicate goalPred = goal;
    String durStr = "durationOf";
    counterfacutal = Factory.createPredicate(counterfacutal.getArgs().get(0).toString());
    if (!counterfacutal.getName().equals("none")) {
      goalPred = Factory.createPredicate("if", counterfacutal, goal);
    }
    double prob = numSuccess / totalExecutions;
    //prob = Math.floor(prob *100) / 100;
    double dur = time.getLeft()/Math.pow(10,3);
    //dur = Math.floor(dur * 100) / 100;
    StateMachine stateMachine = goalManager.getStateMachine();
    stateMachine.assertBelief(Factory.createPredicate("probabilityOf",goalPred.toString(),Double.toString(prob)));
    Predicate d = Factory.createPredicate(durStr, goal.toString(), Double.toString(dur));
    stateMachine.assertBelief(d);
    Symbol probSymbol = Factory.createSymbol(Double.toString(prob));
    Symbol durSymbol = Factory.createSymbol(Double.toString(dur));
    if (mostLikelyStepFailure != null) {
      log.info("simulation results: "
          + "\ngoal: " + goal.toString()
          + "\nsuccess probability: " + numSuccess / totalExecutions + " (" + numSuccess + "/" + totalExecutions + ")"
          + "\nduration (ms): " + (dur) +  " variance: " + (time.getRight() / Math.pow(10,3))
          + "\nmost likely step failure: " + mostLikelyStepFailure + " " + mostLikelyStepFailures.get(mostLikelyStepFailure) / totalExecutions
        //+ "\nhighest failure reasons: " + highestFailureReasons.toString() + " " + executionFailureReasons.get(highestFailureReasons) /  totalExecutions
      );
      stateMachine.assertBelief(Factory.createPredicate("propertyOf", Factory.createPredicate(mostLikelyStepFailure.toString()), Factory.createSymbol("mostLikelyToFail")));
      log.info("Failure locations");
      for (Map.Entry<Predicate, Integer> failureStep : mostLikelyStepFailures.entrySet()) {
        log.info(failureStep.getKey() + " " + failureStep.getValue());
      }
    } else{
      log.info("simulation results: "
                + "\ngoal: " + goal
                + "\nsuccess probability: " + numSuccess / totalExecutions + " (" + numSuccess + "/" + totalExecutions + ")"
                + "\nduration (ms): " + (dur) +  " variance: " + (time.getRight() / Math.pow(10,3))
                + "\nmost likely step failure: none");
    }
    lastProb = probSymbol.clone();
    lastDur = durSymbol.clone();
    if (mostLikelyStepFailure != null) {
      lastFail = mostLikelyStepFailure.clone();
      //List<Symbol> failureArgs = mostLikelyStepFailure.getArgs();
      //failureArgs.add(0, agent);
      //mostLikelyStepFailure = new Predicate(mostLikelyStepFailure.getName(),failureArgs);
    }
    //inserting actor into mostLikelyStepFailure so we have that info for nlg
    return Triple.of(probSymbol, durSymbol, mostLikelyStepFailure);
  }

  /**
   * process the execution trace
   * @param startStep
   * @param current
   * @param startFound
   * @return
   */
  // TODO: understand what is going on and rework / comment
  private static Triple<Long, PerformanceAssessment.ActionTrace, Set<Predicate>> processExecutionTrace(Context startStep, Context current, boolean startFound) {
    // startStep is the initial step during simulation
    // current = top most child context
    if (current.getChildContexts().isEmpty()) {
      if (startFound || current.equals(startStep)) {
        //log.info("step " + current.getSignatureInPredicateForm() + " " + current.getDuration() + " " + current.getStatus() + " " + current.getJustification().getFailureReason());
        PerformanceAssessment.ActionTrace actionTrace = new PerformanceAssessment.ActionTrace(current.getSignatureInPredicateForm(), current.getStatus());
        Set<Predicate> failureReasons = new HashSet<>(current.getJustification().getFailureReason());
        long duration = current.getDuration();
        return Triple.of(duration, actionTrace, failureReasons);
      }
    } else {
      PerformanceAssessment.ActionTrace actionTrace = new PerformanceAssessment.ActionTrace(current.getSignatureInPredicateForm(), current.getStatus());
      Set<Predicate> failureReasons = new HashSet<>();
      long duration = 0;
      if (startFound || current.equals(startStep)) { // all children should have been executed during simulation
        //log.info("step " + current.getSignatureInPredicateForm() + " " + current.getDuration() + " " + current.getStatus() + " " + current.getJustification().getFailureReason());
        for (Context child : current.getChildContexts().getChildrenContexts()) {
          Triple<Long, PerformanceAssessment.ActionTrace, Set<Predicate>> executionInfo = processExecutionTrace(startStep, child, true);
          if (executionInfo != null) { // this should always be the case if the start has been found
            actionTrace.addNode(executionInfo.getMiddle());
            duration += executionInfo.getLeft();
            if (child.isFailure()) {
              failureReasons.addAll(executionInfo.getRight());
              return Triple.of(duration, actionTrace, failureReasons);
            }
          }
        }
        if (current.isFailure()) {
          failureReasons.addAll(current.getJustification().getFailureReason());
        }
        return Triple.of(duration, actionTrace, failureReasons);
      } else {
        for (Context child : current.getChildContexts().getChildrenContexts()) {
          Triple<Long, PerformanceAssessment.ActionTrace, Set<Predicate>> executionInfo = processExecutionTrace(startStep, child, startFound);
          if (executionInfo != null) {
            startFound = true;
            actionTrace.addNode(executionInfo.getMiddle());
            duration += executionInfo.getLeft();
            if (child.isFailure()) {
              failureReasons.addAll(executionInfo.getRight());
              return Triple.of(duration, actionTrace, failureReasons);
            }
          }
        }
        if (startFound) {
          if (current.isFailure()) {
            failureReasons.addAll(current.getJustification().getFailureReason());
          }
          return Triple.of(duration, actionTrace, failureReasons);
        }
      }
    }
    return null;
  }

  /**
   * get the results from the simulation (resulting state, time, action trace, failure reasons) by first getting the resulting state
   * and then processing the actual execution trace
   * @param goal simulation goal -- used to get the root context and the state machine
   * @param startStep start context of the remaining action
   * @return the simulated resulting state
   */
  private static PerformanceAssessment.SimulationResult processGoalSimulation(Goal goal, Context startStep) {
    Context topContext = goal.getRootContext();
    Set<Predicate> terminationState = topContext.getStateMachine().getStateChange(startStep, topContext);
    PerformanceAssessment.SimulationResult simulationResult = null;
    if (terminationState != null && !terminationState.isEmpty()) {
      // filter out all default post-conditions unless the action matches the goal
      terminationState = terminationState.stream().filter(p -> (!(p.getName().equals("succeeded") || p.getName().equals("failed"))
        || p.getArgs().get(0).equals(goal.getPredicate()))).collect(Collectors.toSet());
      Triple<Long, PerformanceAssessment.ActionTrace, Set<Predicate>> executionInfo = processExecutionTrace(startStep, topContext, false);
      simulationResult = new PerformanceAssessment.SimulationResult(terminationState, executionInfo.getMiddle(), executionInfo.getLeft(), executionInfo.getRight());
    }
    return simulationResult;
  }


  private static class SimulateGoal implements Runnable {
    private Goal goal;
    private StateMachine stateMachine;
    private Map<Set<Predicate>, StateDistribution> stateDistributions;
    private int index;

    public SimulateGoal(Goal goal, StateMachine stateMachine, Map<Set<Predicate>, StateDistribution> stateDistributions, int index) {
      this.goal = goal;
      this.stateMachine = stateMachine;
      this.stateDistributions = stateDistributions;
      this.index = index;
    }

    public void run() {
      Pair<Goal, Context> simInfo = executeSimulation(goal, stateMachine);
      SimulationResult simulationResult = processGoalSimulation(simInfo.getLeft(), simInfo.getRight());
      if (simulationResult != null) {
        updateStateDistributions(simulationResult, stateDistributions);
      }
    }
  }

  private static class SimulationResult {
    public final Set<Predicate> resultingState;
    public final ActionTrace actionTrace;
    public final double duration;
    public final Set<Predicate> failureReasons;

    SimulationResult(Set<Predicate> resultingState, ActionTrace actionTrace, double duration, Set<Predicate> failureReasons) {
      this.resultingState = resultingState;
      this.actionTrace = actionTrace;
      this.duration = duration;
      this.failureReasons = failureReasons;
    }
  }

  // keep track of the duration, action trace, and failure reasons for specific state
  private static class StateDistribution {
    //Set<Predicate> state;
    int timesAchieved;
    private double timeMean; // TODO: change this to a probability class
    private double timeVar;
    private double sumSqDiff;
    private List<Double> times;
    List<ActionTrace> actionTraces; // possibly change to set?
    List<Set<Predicate>> failureReasons; // keep track of failure reasons to action traces?

    StateDistribution(double duration, ActionTrace actionTrace, Set<Predicate> failureReasons) {
      this.timeMean = duration;
      actionTraces = new ArrayList<>();
      actionTraces.add(actionTrace);
      this.failureReasons = new ArrayList<>();
      addFailureReasons(failureReasons);
      timesAchieved = 1;
      times = new ArrayList<>();
      times.add(duration);
    }

    void updateDistribution(double duration, ActionTrace actionTrace, Set<Predicate> failureReasons) {
      timesAchieved ++;
      if (timesAchieved > 1) {
        updateTimePriors(duration);
      } else {
        timeMean = duration;
      }
      actionTraces.add(actionTrace);
      addFailureReasons(failureReasons);
    }
    void addFailureReasons(Set<Predicate> failureReasons) {
      if (!failureReasons.isEmpty()) {
        this.failureReasons.add(failureReasons);
      }
    }

    /**
     * incremental update of time distribution after each execution
     * @param time time to complete the action
     */
    private void updateTimePriors(double time) {
      times.add(time);
      double deltaMean = time - timeMean;
      timeMean = timeMean + ((deltaMean) / (double) timesAchieved);
      double deltaNewMean = time - timeMean;
      /*
        Welford's online algorithm
        sumSqDiff = sumSqDiff + (obs - prior) * (obs - newMean)
        var = sumSqDiff / (n-1)
       */
      sumSqDiff = sumSqDiff + (deltaMean) * (deltaNewMean);
      timeVar = sumSqDiff / ((double) timesAchieved);
    }

    double getTime() {
      return timeMean;
    }

    Pair<Double, Double> getTimeInfo() {
      //log.info(times.toString());
      return Pair.of(timeMean, timeVar);
    }

    int getTimesAchieved() {
      return timesAchieved;
    }

    List<ActionTrace> getActionTraces() {
      return actionTraces;
    }

    List<Set<Predicate>> getFailureReasons() {
      return failureReasons;
    }
  }

  // TODO: consider using the state class instead, that way it can keep track of the state between actions
  //       might want to create new ActionTrace class which saves state in tree according to action
  private static class ActionTrace {
    private ArrayList<ActionTrace> children;
    private ActionTrace parent;
    private Predicate actionSignature;
    private ActionStatus actionStatus;

    ActionTrace() {
      children = new ArrayList<>();
      actionSignature = Factory.createPredicate("root");
      parent = null;
    }

    private ActionTrace(Predicate actionSignature, ActionTrace parent) {
      children = new ArrayList<>();
      this.actionSignature = actionSignature;
      this.parent = parent;
    }

    ActionTrace(Predicate actionSignature, ActionStatus status) {
      children = new ArrayList<>();
      this.actionSignature = actionSignature;
      parent = null;
      actionStatus = status;
    }

    void addNewNode(Predicate actionSignature) {
      addNode(new ActionTrace(actionSignature, this));
    }

    void addNode(ActionTrace child) {
      children.add(child);
    }

    List<ActionTrace> getChildActions() {
      return children;
    }

    Predicate getActionSignature() {
      return actionSignature;
    }

    ActionStatus getActionStatus() {
      return actionStatus;
    }

    Predicate getFailureStep() {
      if (actionStatus.isFailure()) {
        if (!children.isEmpty()) {
          for (ActionTrace child : children) {
            if (child.getActionStatus().isFailure()) {
              //Predicate failedStep = child.getFailureStep();
              Predicate failedStep = child.getActionSignature();
              if (failedStep != null) {
                return failedStep;
              }
            }
          }
        }
        return actionSignature;
      }
      return null;
    }
  }

  enum AssessmentType {
    COMPLETE,
    MODIFY,
    NONE,
    STATE;

    public static AssessmentType fromString(String value) {
      return Utilities.strToEnum(AssessmentType.class, value);
    }
  }

}
