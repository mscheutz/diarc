package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry.Builder;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class RLActionManager extends DiarcComponent {
  protected Logger log = LoggerFactory.getLogger(RLActionManager.class);
  int policyId = 0;
  Set<Predicate> state;

  Set<Integer> seenStates;

  public RLActionManager() {
    super();
    state = new HashSet<>();
    seenStates = new HashSet<>();
  }

  private int nextPolicyId() {
    return policyId++;
  }

  @TRADEService
  public int createAction(String name, List<String> args, List<String> preconds, List<String> effects, String executor) {
    log.info("Creating action");
    Builder action = new Builder(name);

    for (String arg : args) {
      ActionBinding.Builder binding = new ActionBinding.Builder("?" + arg, String.class);
      action.addRole(binding.build());
    }
    for (String precond : preconds) {
      action.addCondition(new Condition(Factory.createPredicate(precond), ConditionType.PRE, Observable.FALSE));
    }
    for (String effect : effects) {
      action.addEffect(new Effect(Factory.createPredicate(effect), EffectType.SUCCESS, Observable.FALSE));
    }
    EventSpec.Builder espec = new EventSpec.Builder(EventSpec.EventType.ACTION);

    espec.setCommand("callPolicy");
    int policyId = nextPolicyId();
    espec.addInputArg(String.valueOf(executor)); //todo: Generate random

    action.addEventSpec(espec.build());
    action.build(true);
    return policyId;
  }

  /**
   * Creates an ASL action for the given policy ID and specified preconditions and effects.
   * Used for dynamically generating planning operators that call an RL policy.
   *
   * @param policyId The unique ID of the policy called in the action
   * @param preconds Symbolic preconditions of the policy
   * @param effects  Symbolic effects of the policy
   */
  @TRADEService
  public void createPolicyAction(int policyId, Set<Predicate> preconds, Set<Predicate> effects) {
    log.info("Creating policy action");
    Builder action = new Builder("policy" + policyId);

    EventSpec.Builder espec = new EventSpec.Builder(EventSpec.EventType.ACTION);
    espec.setCommand("callPolicy");
    espec.addInputArg(String.valueOf(policyId));
    action.addEventSpec(espec.build());

    action.addConditions(preconds.stream().map(p -> new Condition(p, ConditionType.PRE, Observable.FALSE)).toList());
    action.addEffects(effects.stream().map(e -> new Effect(e, EffectType.SUCCESS, Observable.FALSE)).toList());

    action.build(true);
  }

  /**
   * Checks the state of the world based on QSR.
   * If the state has not been seen before, create a new policy action going from the previous state to the current state.
   *
   * @param obs Subsymbolic state to observe
   * @return Returns the policy ID corresponding to the current state of the world.
   */
  @TRADEService
  public int checkState(Object obs) {

    //todo: Update consultants based on obs


    Set<Predicate> newState;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("observeQSR"));
      newState = tsi.call(Set.class);
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }

    if (!seenStates.contains(newState.hashCode())) {
      //todo: Update belief?
      log.info("New state entered");
      createPolicyAction(state.hashCode(), state, newState);
      state = newState;
    }
    return state.hashCode();
  }

  @Action
  @TRADEService
  public Justification failureTest() {
    return new ConditionJustification(false);
  }

  @Action
  @TRADEService
  public Justification update_rl() {
    log.info("Recovering");
    return new ConditionJustification(true);
  }
}
