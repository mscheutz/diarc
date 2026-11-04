/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import java.util.HashSet;
import java.util.Locale;
import java.util.Set;


public class ActorAction extends Msg {
  public WorldState preWorldState;
  public WorldState postWorldState;
  public Action action;
  public int entityID;

  public class Action {
    public String action;
    public String args;
    public String result;
    public int stepNumber;
  }

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(generateActorAssertions());
    assertions.addAll(postWorldState.generateAssertions());
    return generateActorAssertions();
  }

  public Set<Predicate> generateActorAssertions() {
    Set<Predicate> assertions = new HashSet<>();

    // action(actor, actionName(arg0,...,argN))
    StringBuilder sb = new StringBuilder("action(");
    sb.append("entity_").append(entityID);
    sb.append(",").append(action.action.toLowerCase(Locale.ROOT)).append("(");
    if (!action.args.isEmpty()) {
      sb.append(String.join(",", action.args.toLowerCase(Locale.ROOT)));
    }
    sb.append("))");
    Predicate p = Factory.createPredicate(sb.toString());
    assertions.add(p);

    return assertions;
  }

}
