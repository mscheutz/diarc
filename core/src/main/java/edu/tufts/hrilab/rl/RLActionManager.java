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

  /**
   * Wraps an executor in a DIARC action and adds it to the database. For use with creating new action from an RL system.
   *
   * @param name     The name of the ASL action you want to create
   * @param args     The arguments to that action. Must be typed if you want to plan with it.
   * @param preconds The symbolic preconditions of that action
   * @param effects  The symbolic effects of that action
   * @param executor The executor you want to wrap, e.g. a python method that calls a policy
   * @return Some unique identifier of the new action. May be deprecated in favor of the name.
   */
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

    espec.setCommand("callPolicy"); // Todo: replace with a function pointer from python, somehow...
    int policyId = nextPolicyId();
    espec.addInputArg(String.valueOf(executor)); //todo: Generate random

    action.addEventSpec(espec.build());
    action.build(true);
    return policyId;
  }

  // Todo: Add observer
}
