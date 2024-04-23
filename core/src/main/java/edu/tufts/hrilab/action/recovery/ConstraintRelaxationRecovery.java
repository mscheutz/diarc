/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.selector.ActionSelector;
import edu.tufts.hrilab.action.selector.GoalPlanningActionSelector;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.*;
import org.apache.commons.lang3.tuple.Pair;

import java.util.*;

public class ConstraintRelaxationRecovery extends GoalRecovery {

  List<Pair<Term, List<Term>>> relaxedConstraints = new ArrayList<>();

  GoalPlanningActionSelector planner;
  ConstraintRelaxationRecovery() {
    planner = new GoalPlanningActionSelector();
  }
  @Override
  public ParameterizedAction selectNextAction(Goal goal, ActionConstraints constraints,
                                              StateMachine stateMachine) {
    ActionSelector selector = ActionSelector.getInstance();
    return selector.selectActionForGoal(goal, constraints, stateMachine);
  }

  @Override
  public boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification,
                                       GoalContext goalContext) {
    if (actionStatus.isFailure()) {
      recoveryStatus = RecoveryStatus.RECOVER;
      return true;
    }

    // Re-assert any constraints relaxed during exploration
    for (Pair<Term, List<Term>> constraint : relaxedConstraints) {
      goalContext.getStateMachine().assertRule(constraint.getLeft(), constraint.getRight());
    }

    recoveryStatus = RecoveryStatus.PLAN;
    return false;
  }

  @Override
  public ParameterizedAction selectRecoveryPolicy(GoalContext goalContext) {
    Goal recoveryGoal = getExplorationGoal(goalContext);
    ParameterizedAction recoveryPolicy = selectNextAction(recoveryGoal, goalContext.getConstraints(), goalContext.getStateMachine());
    return recoveryPolicy;
  }

  private Goal getExplorationGoal(GoalContext goalContext) {
    try {
        //Check if we're current in a violated state
        Justification violated = TRADE.getAvailableService(new TRADEServiceConstraints().name("getViolations").argTypes(Symbol.class,StateMachine.class)).call(Justification.class,
            goalContext.getGoal().getActor(), goalContext.getStateMachine());
        if (violated.getValue()) {
          return generateViolatedGoal(goalContext, violated);
        }
    } catch (TRADEException e) {
      //TODO:brad: not sure this was a sound change...
      log.error("[getExplorationGoal]",e);
//      throw new RuntimeException(e);
    }

    //If we're not in a violated state but we still failed, it's because we can't plan without
    // getting into a
    // violated state.
    return generateRelaxationGoal(goalContext);
  }

  /**
   * Returns a plan to get out of a violated state
   *
   * @param goalContext The current context, which contains a violated state
   * @param violated    The violation we want to get out of
   * @return A new state that, if accomplished, will get out of a violated state
   */
  private Goal generateViolatedGoal(GoalContext goalContext, Justification violated) {
    StateMachine stateMachine = goalContext.getStateMachine();

    for (Term condition : violated.getPredicates()) {

      Pair<Term, List<Term>> rule = stateMachine.getRule(condition.getName()); //Get the rule via
      // its name
      List<Term> goalPreds = new ArrayList<>();

      // Manually insert actor
      Symbol actor = goalContext.getGoal().getActor();
      Map<Variable, Symbol> bindings = violated.getBindings().get(0);
      bindings.put(Factory.createVariable("X"), actor);

      for (Term precond : rule.getRight()) {
        Term newGoal = precond.copyWithNewBindings(bindings);

        // Flip value (to get OUT of the state)
        if (newGoal.isNegated()) {
          newGoal = newGoal.toUnnegatedForm();
        } else {
          newGoal = newGoal.toNegatedForm();
        }
        goalPreds.add(newGoal);
      }
      //If any of the preconditions for the violated are negated, we're out of a violated state
      Predicate goalPred = Factory.createPredicate("or", goalPreds);
      Goal g = new Goal(goalContext.getGoal().getActor(), goalPred);

      //todo: pass with bindings and only check name there
      //plan to get out of each of the conditions in the rule, using bindings from the justification
      stateMachine.retractRule(condition);
      if (planner.selectActionForGoal(g, goalContext.getConstraints(), stateMachine) != null) {
        return g;
      }
    }
    return null;
  }

  /**
   * Currently, generates a bound negotiateConstraints action, using all valid relaxations
   *
   * @param goalContext
   * @return
   */
  private Goal generateRelaxationGoal(GoalContext goalContext) {

    List<Symbol> successfulRelaxations = new ArrayList<>();
    StateMachine stateMachine = goalContext.getStateMachine();

    for (Pair<Term, List<Term>> rule : stateMachine.getRules()) {
      //Get all valid relaxations
      if (!rule.getLeft().getName().contains("Violation")) {
        continue;
      }
      //temporarily retract the rule to test if this is the rule we're violated
      stateMachine.retractRule(rule.getLeft(), rule.getRight());

      //Test if the state is plannable without retracted rule
      if (planner.selectActionForGoal(goalContext.getGoal(), goalContext.getConstraints(),
          stateMachine) != null) {
        successfulRelaxations.add(Factory.createSymbol(rule.getLeft().getName()));
      }
      //re-assert the rule for further tests
      stateMachine.assertRule(rule.getLeft(), rule.getRight());
    }

    if (successfulRelaxations.isEmpty()) {
      return null;
    }

    // Create a composite goal that provides all relaxable constraints
    Goal g = new Goal(Factory.createPredicate("negotiateConstraints",
        Factory.createSymbol("self"), Factory.createPredicate("options", successfulRelaxations)));
    return g;
  }
}
