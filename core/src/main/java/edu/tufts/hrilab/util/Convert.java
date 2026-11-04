/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Methods for converting between various Transform related types (points, orientations, etc).
 *
 * NOTE: this is similar to the edu.tufts.hrilab.diarcros.util.Convert class but contains only non-ROS versions.
 */
public class Convert {
  public static final int ROLL = 0;
  public static final int PITCH = 1;
  public static final int YAW = 2;

  /**
   * Converts a point and orientation to matrix format.
   *
   * @param p point (x,y,z)
   * @param q quaternion (x,y,z,w)
   * @return a Matrix4d representing the point and orientation
   */
  public static Matrix4d convertToMatrix4d(Point3d p, Quat4d q) {
    Matrix4d tmp = new Matrix4d();
    tmp.setIdentity();
    tmp.setRotation(q);
    tmp.setTranslation(new Vector3d(p.x, p.y, p.z));
    return tmp;
  }

  /**
   * converts a point and orientation to matrix format
   *
   * @param pose
   * @return a Matrix4d representing the point and orientation
   */
  public static Matrix4d convertToMatrix4d(Pair<Point3d, Quat4d> pose) {
    return convertToMatrix4d(pose.getLeft(), pose.getRight());
  }

  /**
   * gets pose from a Matrix4d
   *
   * @param m matrix to get pose from
   * @return pose represented by m
   */
  public static Pair<Point3d, Quat4d> convertToPair(Matrix4d m) {
    Vector3d v = new Vector3d();
    m.get(v);
    Quat4d q = new Quat4d();
    q.set(m);
    return new MutablePair<>(new Point3d(v.x, v.y, v.z), q);
  }

  /**
   * Convert a quaternion to euler angles.
   * quat4d -> XYZ Euler Angles (NASA Standard Aeroplane) (https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/indexLocal.htm)
   *
   * @param q Quat4d quaternion
   * @return
   */
  public static double[] convertToEuler(Quat4d q) {
    double sqw = Math.pow(q.w, 2);
    double sqx = Math.pow(q.x, 2);
    double sqy = Math.pow(q.y, 2);
    double sqz = Math.pow(q.z, 2);
    //heading/yaw is y in NASA SA
    double heading = Math.atan2(2 * (q.x * q.y + q.z * q.w), (sqx - sqy - sqz + sqw));
    //bank/roll is x in NASA SA
    double bank = Math.atan2(2 * (q.y * q.z + q.x * q.w), (-sqx - sqy + sqz + sqw));
    //attitude/pitch is z in NASA SA
    double attitude = Math.asin(-2 * (q.x * q.z - q.y * q.w) / (sqx + sqy + sqz + sqw));
    double[] euler = new double[3];
    euler[ROLL] = bank;
    euler[PITCH] = attitude;
    euler[YAW] = heading;
    return euler;
  }

  /**
   * Convert euler angles to Quat4d quaternion.
   *
   * @param yaw
   * @param pitch
   * @param roll
   * @return
   */
  public static Quat4d convertToQuat4d(double yaw, double pitch, double roll) {
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
