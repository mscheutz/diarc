/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.stm;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Describes a position from which an object can be grasped. Not guaranteed to
 * be valid for any robot.
 *
 * @author Linc Berkley
 */
public class Grasp implements Serializable {

  public enum Type {
    PINCH_TOGETHER, PINCH_APART, PUSH, TWO_ARM, UNKNOWN
  };

  private static Logger log = LoggerFactory.getLogger(Grasp.class);

  private Type type = Type.UNKNOWN;
  
  /**
   * Cached enum values for converting int to enum from native code.
   */
  private final Type[] type_values = Type.values();

  /**
   * Points to place fingertips. 1 element for PINCH_TOGETHER or PUSH, 2
   * elements for PINCH_APART or TWO_ARM.
   */
  private List<Vector3d> points = new ArrayList<>();

  /**
   * Orientation for each gripper. 1 element for PINCH_TOGETHER, PINCH_APART, or
   * PUSH; 2 elements for TWO_ARM.
   */
  private List<Quat4d> orients = new ArrayList<>();

  public Grasp() {
  }

  public Grasp(Point3d point, Quat4d orient) {
    this.type = Type.PINCH_TOGETHER;
    points.add(new Vector3d(point));
    orients.add(orient);
  }

  public Grasp(Type type, List<Vector3d> points, List<Quat4d> orients) {
    this.type = type;
    this.points = points;
    this.orients = orients;
  }
  
  public void setType(Type grasp_type) {
    type = grasp_type;
  }
  
  public void setType(int grasp_ordinal) {
    type = type_values[grasp_ordinal];
  }

  public void setPoint(int index, double x, double y, double z) {
    if (index < 0 || index > 1) {
      log.error("[setPoint] Trying to set point with index out of range: " + index);
    }

    Vector3d point = new Vector3d(x, y, z);
    setPoint(index, point);
  }

  public void setPoint(int index, Vector3d point) {
    if (points.size() > index) {
      this.points.set(index, point);
    } else {
      this.points.add(index, point);
    }
  }

  public void setOrientation(int index, double x, double y, double z, double w) {
    if (index < 0 || index > 1) {
      log.error("[setOrientation] Trying to set orientation with index out of range.");
      return;
    }

    Quat4d orient = new Quat4d(x, y, z, w);
    setOrientation(index, orient);
  }

  public void setOrientation(int index, Quat4d orient) {
    if (orients.size() > index) {
      this.orients.set(index, orient);
    } else {
      this.orients.add(index, orient);
    }
  }

  public Type getType() {
    return type;
  }

  public int getNumPoints() {
    return points.size();
  }

  public Vector3d getPoint(int index) {
    if (index < 0 || index >= points.size()) {
      log.error("[getPoint] Trying to get point with index out of range.");
      return null;
    }

    return points.get(index);
  }

  public int getNumOrientations() {
    return orients.size();
  }

  public Quat4d getOrientation(int index) {
    if (index < 0 || index >= orients.size()) {
      log.error("[getOrientation] Trying to get orientation with index out of range.");
      return null;
    }

    return orients.get(index);
  }
  
}
