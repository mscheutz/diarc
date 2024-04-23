/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import java.util.HashSet;
import java.util.Set;

public class CommandResult extends Msg {
  public String command;
  public String argument;
  public String result;
  public String message;
  public double stepCost;

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.add(generateStepCostAssertion());
    return assertions;
  }

  public Predicate generateStepCostAssertion() {
    String cmd = command.toLowerCase();
    if (cmd.equals("craft")) {
      String craftedItem = message;
      String[] craftedItemSplit = craftedItem.split(" ");
      craftedItem = craftedItemSplit[craftedItemSplit.length - 1];
      craftedItem = craftedItem.replace("minecraft:", "").replace("polycraft:", "");
      cmd = cmd + craftedItem;
    }
    Predicate costFunction = Factory.createPredicate("cost_1", cmd);
    return Factory.createPredicate("fluent_equals", costFunction, Factory.createSymbol(Double.toString(stepCost)));
  }
}
