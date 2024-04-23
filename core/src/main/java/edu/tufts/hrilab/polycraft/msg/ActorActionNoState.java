/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashSet;
import java.util.Locale;
import java.util.Set;


public class ActorActionNoState extends Msg {
  public Action[] actions;
  public int entityID;

  public class Action {
    public String action;
    public String args;
    public String result;
    public int stepNumber;
    public int[] pos;
    public String dir;
    public BlockInFront blockInFront;
  }

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(generateActorAssertions());
    return generateActorAssertions();
  }

  public Set<Predicate> generateActorAssertions() {
    Set<Predicate> assertions = new HashSet<>();

    for (Action action : actions) {
      // action(step, actionName(arg0,...,argN))
      StringBuilder sb = new StringBuilder("action(");
      sb.append(action.stepNumber).append(",");
      String actionString;
      if (!action.args.isEmpty()) {
        String argItem = SymbolResolver.toGridItem(action.args);
        actionString = SymbolResolver.toAction(action.action, argItem);
        if (actionString!=null) {
          sb.append(actionString).append("(").append(argItem).append("))");
          Predicate p = Factory.createPredicate(sb.toString());
          assertions.add(p);
        }
      } else {
        String argItem = SymbolResolver.toGridItem(action.blockInFront.name, action.blockInFront.variant);
        actionString = SymbolResolver.toAction(action.action, argItem);
        if (actionString!=null) {
          sb.append(actionString).append("(").append(argItem).append("))");
          Predicate p = Factory.createPredicate(sb.toString());
          assertions.add(p);
        }
      }
    }

    return assertions;
  }

}
