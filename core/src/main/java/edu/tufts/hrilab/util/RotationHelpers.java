/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.util;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

/**
 * A collection of static methods to help out with rotations.
 *
 */
public class RotationHelpers {
  /**
   * Converts a Tuple containing a rotation about x, y, and z (to be
   * applied in that order) into an equivalent quaternion.
   *
   * @param xyz The three desired rotations
   * @return The same rotation as a quaternion.
   */
  public static Quat4d xyzRotationsToQuaternion(Tuple3d xyz) {
    Rotation rot = new Rotation(RotationOrder.XYZ, xyz.x, xyz.y, xyz.z);
    return new Quat4d(-rot.getQ1(), -rot.getQ2(), -rot.getQ3(), rot.getQ0());  // inverting the imaginary outputs, see Javadocs for Rotation for an explanation of why... or perhaps better, just take my word for it and leave these negatives
  }

  /**
   * Converts a Tuple containing a rotation about x, y, and z (to be
   * applied in that order) into a matrix that can rotate points by x,
   * then y, and then z.
   *
   * @param xyz The three desired rotations.
   * @return A matrix that achieves them.
   */
  public static Matrix3d xyzRotationsToRotationMatrix(Tuple3d xyz) {
    Matrix3d mat = new Matrix3d();
    mat.set(xyzRotationsToQuaternion(xyz));
    return mat;
  }

  /**
   * Converts a Tuple containing a rotation about x, y, and z (to be
   * applied in that order) into a matrix that can rotate points by x,
   * then y, and then z.
   *
   * The translate component of the matrix will always be zero.
   *
   * @param xyz The three desired rotations.
   * @return A matrix that achieves them.
   */
  public static Matrix4d xyzRotationsToTransformMatrix(Tuple3d xyz) {
    Matrix4d mat = new Matrix4d();
    mat.setIdentity();
    mat.setRotation(xyzRotationsToQuaternion(xyz));
    return mat;
  }
  
  public static Tuple3d quatToXYZRotations(Quat4d q) {
	double ysqr = q.y * q.y;
    
    // roll (x-axis rotation)
	double t0 = +2.0 * (q.w * q.x + q.y * q.z);
	double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
	double x = Math.atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q.w * q.y - q.z * q.x);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	double y = Math.asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q.w * q.z + q.x * q.y);
	double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);  
	double z = Math.atan2(t3, t4);
    
    Tuple3d returnAngles = new Point3d(x,y,z);
    return returnAngles;
  }
  

  /**
   * Converts a translation and a rotation into a transform matrix that
   * can be applied to points.
   *
   * @param translation How far the points should move.
   * @param rotation    How far the points should rotate.
   * @return
   */
  public static Matrix4d buildTransformMatrix(Vector3d translation, Tuple3d rotation) {
    return new Matrix4d(xyzRotationsToQuaternion(rotation), translation, 1);
  }

  public static Quat4d rotateQuaternion(Quat4d toRotate, String axis, double rad) {
      Quat4d rot;

      double a = Math.sin(rad/2.0);
      double w = Math.cos(rad/2.0);

      switch (axis) {
        case "x":
            rot = new Quat4d(a, 0, 0, w);
            break;
        case "y":
            rot = new Quat4d(0, a, 0, w);
            break;
        case "z":
            rot = new Quat4d(0, 0, a, w);
            break;
        default:
            return null;
    }
    rot.normalize();
    toRotate.mul(rot);
    return toRotate;
  }

    /**
     * Creates a quaternion based off of a given axis of rotation and given radians of rotation
     *
     * @param axis_of_rot The axis of rotation
     * @param rev_rad     The magnitude of the rotation in radians
     * @return A quaternion based off of a given axis of rotation and given radians of rotation
     */
    public static Quat4d createQuaternion(Vector3d axis_of_rot, double rev_rad) {
        axis_of_rot.normalize();

        double sin_half_angle = Math.sin(rev_rad / 2);
        double cos_half_angle = Math.cos(rev_rad / 2);

        Quat4d quaternion = new Quat4d();
        quaternion.x = axis_of_rot.x * sin_half_angle;
        quaternion.y = axis_of_rot.y * sin_half_angle;
        quaternion.z = axis_of_rot.z * sin_half_angle;
        quaternion.w = cos_half_angle;

        return quaternion;
    }

    /**
     * Gives a quaternion representing the "mirror" orientation of the given quaternion.
     * While it is not possible to directly mirror a quaternion, we can view the resultant quaternion as a 180-degree
     * rotation of the reference quaternion around an axis orthogonal to its equivalent normal vector
     *
     * @param ref_orient The reference orientation given by the inputted quaternion
     * @return The "mirror" orientation of the given quaternion
     */
    public static Quat4d mirrorOrientation(Quat4d ref_orient) {
        Vector3d normal_vect = quaternionToNormVect(ref_orient);

        // 1. Find an axis of rotation orthogonal to the normal vector by projecting the z axis onto the normal vector's plane.
        //    If the normal vector and z axis are parallel, we can project the x-axis vector instead
        Vector3d z_axis = new Vector3d(0, 0, 1);
        Vector3d x_axis = new Vector3d(1, 0, 0);
        Vector3d axis_of_rot = new Vector3d();
        if (z_axis.epsilonEquals(normal_vect, 1e-6)) { // If the normal vector is similar to the z axis
            axis_of_rot = SimpleGeometry.projectVectorOntoPlane(x_axis, normal_vect); // project x-axis instead
        } else {
            axis_of_rot = SimpleGeometry.projectVectorOntoPlane(z_axis, normal_vect);
        }

        // 2. Construct a quaternion representing a 180 rotation around that axis
        Quat4d mirror_rot = createQuaternion(axis_of_rot, Math.PI);

        // 3. Combine that quaternion with the reference quaternion. This should result in the "mirrored" orientation
        Quat4d mirror_orient = new Quat4d();
        mirror_orient.mul(mirror_rot, ref_orient);

        return mirror_orient;
    }

    /**
     * Converts an orientation quaternion into it's equivalent normal vector.
     * We can think of that normal vector as the rotational transformation
     * applied by the given quaternion onto the base vector (the unit x-axis vector)
     *
     * @param orient_quat the orientation quaternion to convert
     * @return the equivalent normal vector
     */
    public static Vector3d quaternionToNormVect(Quat4d orient_quat) {
        Vector3d base_vect = new Vector3d(1, 0, 0);
        Vector3d normal_vect = new Vector3d();
        Matrix4d tmp = new Matrix4d();
        tmp.setIdentity();
        tmp.setRotation(orient_quat);
        tmp.setTranslation(new Vector3d(0,0,0));
        tmp.transform(base_vect, normal_vect);
        normal_vect.normalize(); // This should already be normal but for good measure
        return normal_vect;
    }

  /**
   * Move point by offset amount in direction of specified offset_axis.
   *
   * @param point       3D point
   * @param orientation point's orientation
   * @param offset_axis direction to apply offset
   * @param offset      (amount to translate along offset_axis)
   * @return
   */
  public static Point3d calcTargetOffset(final Point3d point, final Quat4d orientation, Vector3d offset_axis, float offset) {
    Matrix4d pose = new Matrix4d(orientation, new Vector3d(point), 1.0);
    Vector3d axis = new Vector3d(offset_axis);
    axis.normalize();
    axis.scale(offset);
    Matrix4d mat = new Matrix4d(new Quat4d(0.0, 0.0, 0.0, 1.0), axis, 1.0);
    Vector3d target_point = new Vector3d();
    pose.mul(mat);
    pose.get(target_point);
    return new Point3d(target_point);
  }
}
