/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.moveit.util;

import edu.tufts.hrilab.diarcros.msg.geometry_msgs.*;

import javax.vecmath.*;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import java.lang.Math;

/**
 * This is a helper class to convert standard java to/from rosjava representations of points, etc.
 * I found I was often writing 'someRosJavaFunction(new Point(point3d.x, point3d.y, point3d.z)',
 * now it's just 'someRosJavaFunction(convert(point3d))'.
 * <p>
 * From there, it grew to things like euler to quaternions, points and quaternions to matrices, etc.
 * This implementation is kinda a mess, but I think it makes the end result more readable.
 */
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

    // vecmath -> geometry_msgs
    public static Point convertToPoint3d(Point3d p) {
        return new Point(p.x, p.y, p.z);
    }

    // geometry_msgs -> vecmath
    public static Point3d convertToPoint3d(Point p) {
        return new Point3d(p.getX(), p.getY(), p.getZ());
    }

    // geometry_msgs -> vecmath
    public static Quat4d convertToQuat4d(Quaternion q) {
        return new Quat4d(q.getX(), q.getY(), q.getZ(), q.getW());
    }

    // vecmath -> geometry_msgs
    public static Quaternion convertToQuaternion(Quat4d q) {
        return new Quaternion(q.x, q.y, q.z, q.w);
    }

    // vecmath -> geometry_msgs
    public static Vector3 convertToVector3(Vector3d v) {
        return new Vector3(v.x, v.y, v.z);
    }

    // geometry_msgs -> vecmath
    public static Vector3d convertToVector3d(Vector3 v) {
        return new Vector3d(v.getX(), v.getY(), v.getZ());
    }

    // TransformStamped -> Transform -> Matrix with rotation, translational components
    public static Matrix4d convertToMatrix4d(TransformStamped t) {
        Matrix4d tmp = new Matrix4d();
        tmp.setIdentity();
        tmp.setRotation(convertToQuat4d(t.getTransform().getRotation()));
        tmp.setTranslation(convertToVector3d(t.getTransform().getTranslation()));
        return tmp;
    }

    public static Pose mulPoses(Pose a, Pose b) {
        Matrix4d matA = convertToMatrix4d(a);
        matA.mul(convertToMatrix4d(b));
        return convertToPose(matA);
    }

    public static Matrix4d convertToMatrix4d(Point3d p, Quat4d q) {
        Matrix4d tmp = new Matrix4d();
        tmp.setIdentity();
        tmp.setRotation(q);
        tmp.setTranslation(new Vector3d(p.x, p.y, p.z));
        return tmp;
    }

    public static Matrix4d convertToMatrix4d(Pose p) {
        Matrix4d tmp = new Matrix4d();
        tmp.setIdentity();
        Quaternion orient = p.getOrientation();
        tmp.setRotation(new Quat4d(orient.getX(), orient.getY(), orient.getZ(), orient.getW()));
        Point point = p.getPosition();
        tmp.setTranslation(new Vector3d(point.getX(), point.getY(), point.getZ()));
        return tmp;
    }

    /* Convenience methods: Make a new pose from whatever type of point and quaternion. */
    public static Pose convertToPose(Point3d p, Quat4d q) {
        return new Pose(convertToPoint3d(p), convertToQuaternion(q));
    }

    public static Pair<Point3d, Quat4d> convertToPose(Pose pose) {
        Point pt = pose.getPosition();
        Quaternion qt = pose.getOrientation();
        return new MutablePair<>(convertToPoint3d(pt), convertToQuat4d(qt));
    }

    public static Pose convertToPose(Matrix4d m) {
        Vector3d v = new Vector3d();
        m.get(v);
        Quat4d q = new Quat4d();
        // m.get(q); EAK: there's a bug in java8 Matrix4d.get(Quat4d) that results in occasional NaNs after sqrt of a negative
        q.set(m);
        return new Pose(new Point(v.x, v.y, v.z), new Quaternion(q.x, q.y, q.z, q.w));
    }

    /* Convenience methods: Make a new transform from whatever type of vector and quaternion. */
    // Plain old transform -> TransformStamped
    public static TransformStamped convertToTransformStamped(Transform t) {
        TransformStamped tmp = new TransformStamped();
        tmp.setTransform(t);
        return tmp;
    }

    public static TransformStamped convertToTransformStamped(Vector3 v, Quaternion q) {
        return convertToTransformStamped(new Transform(v, q));
    }

    public static TransformStamped convertToTransformStamped(Vector3 v, Quat4d q) {
        return convertToTransformStamped(new Transform(v, convertToQuaternion(q)));
    }

    public static TransformStamped convertToTransformStamped(Vector3d v, Quaternion q) {
        return convertToTransformStamped(new Transform(convertToVector3(v), q));
    }

    public static TransformStamped convertToTransformStamped(Vector3d v, Quat4d q) {
        return convertToTransformStamped(new Transform(convertToVector3(v), convertToQuaternion(q)));
    }

}
