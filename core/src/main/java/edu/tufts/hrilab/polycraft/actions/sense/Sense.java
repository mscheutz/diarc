/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.polycraft.actions.GameAction;

import java.util.HashSet;
import java.util.Set;

public abstract class Sense extends GameAction {

  @Override
  public boolean canChangeGameState() {
    return false;
  }

  public Set<Predicate> getLocationAssertions(boolean includeAir) {
    return new HashSet<>();
  }
}
