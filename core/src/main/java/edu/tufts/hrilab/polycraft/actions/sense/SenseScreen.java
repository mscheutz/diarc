/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import edu.tufts.hrilab.fol.Predicate;

import java.util.*;

public class SenseScreen extends Sense {
  protected String response;

  @Override
  public String getCommand() {
    return "SENSE_SCREEN";
  }

  @Override
  public boolean getSuccess() {
    return true;
  }

  @Override
  public double getStepCost() {
    return 0;
  }

  @Override
  protected void parseResponse() {
  }

  @Override
  public Set<Predicate> getAssertions() {
    return null;
  }

}
