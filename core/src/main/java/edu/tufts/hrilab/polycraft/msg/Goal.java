/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import java.util.HashSet;
import java.util.Set;

public class Goal extends Msg {
  public String goalType;
  public boolean goalAchieved;
  public String Distribution;

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.add(generateNoveltySignalAssertion());
    return assertions;
  }

  public Predicate generateNoveltySignalAssertion() {
    // get one-off preds
    log.debug("Novelty signal: " + Distribution);
    if (Distribution.equals("Novelty")) {
      return Factory.createPredicate("noveltySignal", "true");
    } else {
      return Factory.createPredicate("not(noveltySignal(true))");
    }
  }
}
