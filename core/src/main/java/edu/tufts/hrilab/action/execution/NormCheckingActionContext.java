/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.rollout.RolloutActionInterpreter;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.*;

import static edu.tufts.hrilab.action.rollout.RolloutActionInterpreter.createRolloutInterpreterFromExecutionTree;

public class NormCheckingActionContext extends ActionContext {
  public NormCheckingActionContext(Context caller, StateMachine sm, ActionDBEntry action, List<? extends Object> arguments, Symbol actor) {
    super(caller, sm, action, arguments, new ArrayList<>(), actor);
  }

  public NormCheckingActionContext(Context caller, StateMachine sm, ActionDBEntry action, Map<String, Object> bindings, ExecutionType executionType, Symbol actor) {
    super(caller, sm, action, bindings, executionType, actor);
  }

  @Override
  public void doStep() {
    if (isScript()) {
      super.doStep();
    } else if (getExecType().shouldExecute()) {    // Check that action is not simulated
      if (!checkViolations()) {
        Justification executionJustification = getDBE().executeAction(collectArguments());
        if (executionJustification.getValue()) {
          redistributeArguments();
        } else {
          // Sets status to fail and exits
          setStatus(ActionStatus.FAIL, executionJustification);
        }
      } else {
        log.warn("Norm violation detected. Not executing action.");
//                TODO(dkasenberg) figure out how to make this better
        Justification justification = new ConditionJustification(false, Factory.createPredicate("violated_norms", (Symbol) this.getArgumentValue("?actor")));
        setStatus(ActionStatus.FAIL, justification);
      }
    } else {
      log.debug("Skipping action call (simulation or observation)");
    }
  }

  private static void makeContextTreeSimulation(Context treeRoot) {
    treeRoot.setExecType(ExecutionType.SIMULATE_ACT);
    for (Context c : treeRoot.getChildContexts().getChildrenContexts()) {
      makeContextTreeSimulation(c);
    }
  }

  public boolean checkViolations() {
//        This prevents an infinite loop of NormCheckingActionContexts creating NormCheckingActionContexts
    if (this.isSimulation()) return false;

//        TODO(dkasenberg) make this a general parameter so that we can use other environments.

    List<String> checkableActions = Arrays.asList("goEast", "goWest", "goNorth", "goSouth", "interactWithObject",
            "toggleShoppingCart");
    if (!checkableActions.contains(this.getDBE().getName())) {
      return false;
    }

//        Simulation should reset its world state to the current state
//        Try to perform a rollout in the simulated world; if it violates the norms, go to MCTS mode
    Predicate goalPred = getSignatureInPredicateForm();
    Goal rolloutGoal = new Goal(getActor("?actor"), goalPred, Observable.FALSE);
    Context newThis = createSimulatedEquivalent(false);

    //Get the observation from the real agent, and set the simagent's observation to it
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("setSimObservation")).call(void.class);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("enableSim")).call(void.class); //enable simulation execution
    } catch (TRADEException te) {
      log.error("[checkViolations]", te);
    }

    RolloutActionInterpreter rolloutInterpreter =
            createRolloutInterpreterFromExecutionTree(rolloutGoal, newThis, 1);
    rolloutInterpreter.call();

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("disableSim")).call(void.class); //disable simulation execution
    } catch (TRADEException te) {
      log.error("[checkViolations]", te);
    }

    try {
      boolean violated = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport").argTypes(Term.class)).call(Boolean.class, Factory.createPredicate("violated_norms(X)"));
//            violated = true;
      if (violated) {
        log.info("NORMS VIOLATED!");
      }

      return violated;
    } catch (TRADEException te) {
      log.error("[checkViolations]", te);
    }

//        ActionDBEntry action = Database.getAction(getDBE().getName(), "agent1", getDBE().getRequiredRolesTypes());
//        action.executeAction(collectArguments());
    return false;
  }
}
