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

  private static Logger log = LoggerFactory.getLogger(Grasp.class);

  /**
   * Points to place fingertips. 1 element for PINCH_TOGETHER or PUSH, 2
   * elements for PINCH_APART.
   */
  private List<Vector3d> points = new ArrayList<>();

  /**
   * Orientation of the gripper pose.
   */
  private Quat4d orientation = null;

  public Grasp() {
  }

  public Grasp(Point3d point, Quat4d orientation) {
    this.points.add(new Vector3d(point));
    this.orientation = orientation;
  }

  public Grasp(List<Vector3d> points, Quat4d orientation) {
    this.points = points;
    this.orientation = orientation;
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

  public void setOrientation(double x, double y, double z, double w) {
    Quat4d orient = new Quat4d(x, y, z, w);
    setOrientation(orient);
  }

  public void setOrientation(Quat4d orientation) {
    this.orientation = orientation;
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

  public Quat4d getOrientation() {
    return orientation;
  }
  
}
