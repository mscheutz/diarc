/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.pose;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;

public class RWSPoseReference extends Reference {
  String pose;

  public RWSPoseReference(Symbol ref, Variable variable) {
    super(ref, variable);
  }

  public RWSPoseReference(Symbol ref, Variable variable, List<Term> properties, String pose) {
    super(ref, variable, properties);
    this.pose = pose;
  }

  public String getPose() {
    return pose;
  }

  public void setPose(String pose) {
    this.pose = pose;
  }

  public boolean hasPose() {
    return pose != null;
  }

  @Override
  public String toString() {
    return super.toString() + " pose = " + ((pose == null) ? " (is null)" : pose);
  }
}
