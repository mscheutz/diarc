/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashSet;
import java.util.Set;

public class Player extends Msg {
  public int[] pos;
  public String facing;
  public double yaw;
  public double pitch;

  @Override
  public Set<Predicate> generateAssertions() {
    return generatePlayerAssertions();
  }

  //at(self, x, y) and facing(self, dir)
  public Set<Predicate> generatePlayerAssertions() {
    Symbol actor = new Symbol("self");

    String[] posStr = SymbolResolver.toGridPos(pos);
    String dir = facing;
    Predicate posPred = Factory.createPredicate("at", actor, new Symbol(posStr[0]), new Symbol(posStr[1]));
    Predicate dirPred = Factory.createPredicate("facing", actor, new Symbol(dir.toLowerCase()));
    log.debug("Adding player at " + posPred);
    log.debug("Adding player facing " + dirPred);
    Set<Predicate> assertions = new HashSet<>();
    assertions.add(posPred);
    assertions.add(dirPred);
    return assertions;
  }
}
