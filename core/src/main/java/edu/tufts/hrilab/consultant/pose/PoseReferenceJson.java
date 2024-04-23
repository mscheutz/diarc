/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.consultant.pose;

import edu.tufts.hrilab.fol.Term;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.List;
import java.util.stream.Collectors;

public class PoseReferenceJson {
  protected String refId;
  protected String variable;
  protected List<String> properties;
  protected double[] position;
  protected double[] orientation;

  public PoseReferenceJson(PoseReference poseReference) {
    this.refId = poseReference.refId.toString();
    this.variable = poseReference.variable.toString();
    this.properties = poseReference.properties.stream().map(Term::toString).collect(Collectors.toList());
    this.position = new double[]{poseReference.position.x, poseReference.position.y, poseReference.position.z};
    this.orientation = new double[]{poseReference.orientation.x, poseReference.orientation.y, poseReference.orientation.z, poseReference.orientation.w};
  }

  public Point3d getPosition() {
    return new Point3d(position[0], position[1], position[2]);
  }

  public Quat4d getOrientation() {
    return new Quat4d(orientation[0], orientation[1], orientation[2], orientation[3]);
  }

  public String getRefId() {
    return refId;
  }

  public String getVariable() {
    return variable;
  }

  public List<String> getProperties() {
    return properties;
  }

  public boolean hasPose() {
    return orientation != null && position != null;
  }

}
