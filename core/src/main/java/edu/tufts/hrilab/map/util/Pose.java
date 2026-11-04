/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map.util;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public class Pose {
  private Point3d position;
  private Quat4d orientation;

  public Pose(Point3d pos, Quat4d rot) {
    position = pos;
    orientation = rot;
  }

  public Pose() {
    position = new Point3d(0, 0, 0);
    orientation = new Quat4d(0, 0, 0, 1);
  }

  public Point3d getPosition() {
    return position;
  }

  public Quat4d getOrientation() {
    return orientation;
  }
}
