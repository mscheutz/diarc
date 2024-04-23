/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.consultant.pose;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.List;

public class PoseReference extends Reference {
  protected Point3d position;
  protected Quat4d orientation;
  protected Symbol poseName; //this is the descriptor used to denote this pose

  public PoseReference(Symbol ref, Variable variable) {
    super(ref, variable);
  }

  protected PoseReference(Symbol ref, Variable variable, List<Term> properties, Point3d position, Quat4d orientation) {
    super(ref, variable, properties);
    this.position = position;
    this.orientation = orientation;
  }

  public Point3d getPosition() {
    return position;
  }

  public Quat4d getOrientation() {
    return orientation;
  }

  public Pair<Point3d, Quat4d> getPose() {
    return new ImmutablePair<>(position, orientation);
  }

  public void setPosition(Point3d position) {
    this.position = position;
  }

  public void setOrientation(Quat4d orientation) {
    this.orientation = orientation;
  }

  public void setPose(Point3d position, Quat4d orientation) {
    this.position = position;
    this.orientation = orientation;
  }

  public boolean hasPose() {
    return orientation != null && position != null;
  }
}
