/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class Convert {
  // quat4d -> YZX Euler Angles (NASA Standard Aeroplane) (https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/indexLocal.htm)
  public static Point3d convertToEuler(Quat4d q) {
      double sqw = Math.pow(q.w, 2);
      double sqx = Math.pow(q.x, 2);
      double sqy = Math.pow(q.y, 2);
      double sqz = Math.pow(q.z, 2);
      //heading is y in NASA SA
      double heading = Math.atan2(2 * (q.x*q.y + q.z*q.w), (sqx - sqy - sqz + sqw));
      //bank is x in NASA SA
      double bank = Math.atan2(2 * (q.y*q.z + q.x*q.w), (-sqx - sqy + sqz + sqw));
      //attitude is z in NASA SA
      double attitude = Math.asin(-2 * (q.x*q.z - q.y*q.w)/(sqx + sqy + sqz + sqw));
      return new Point3d(bank, heading, attitude);
  }

  public static Matrix4d convertToMatrix4d(Point3d p, Quat4d q) {
      Matrix4d tmp = new Matrix4d();
      tmp.setIdentity();
      tmp.setRotation(q);
      tmp.setTranslation(new Vector3d(p.x, p.y, p.z));
      return tmp;
  }

  public static Quat4d eulerToQuat4d(double yaw, double pitch, double roll) {
      double cy = Math.cos(yaw * 0.5);
      double sy = Math.sin(yaw * 0.5);
      double cp = Math.cos(pitch * 0.5);
      double sp = Math.sin(pitch * 0.5);
      double cr = Math.cos(roll * 0.5);
      double sr = Math.sin(roll * 0.5);

      Quat4d q = new Quat4d();
      q.setW(cr * cp * cy + sr * sp * sy);
      q.setX(sr * cp * cy - cr * sp * sy);
      q.setY(cr * sp * cy + sr * cp * sy);
      q.setZ(cr * cp * sy - sr * sp * cy);

      return q;
  }
}
