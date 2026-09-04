/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.util;

import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Transform;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.TransformStamped;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import java.lang.Math;

/**
 * This is a helper class to convert standard java to/from rosjava representations of points, etc.
 * I found I was often writing 'someRosJavaFunction(new Point(point3d.x, point3d.y, point3d.z)',
 * now it's just 'someRosJavaFunction(convert(point3d))'. Also some functionality for converting
 * to/from quaternions/euler, etc.
 */
public class Convert {
    public static final int ROLL  = 0;
    public static final int PITCH = 1;
    public static final int YAW   = 2;

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
        Quat4d q = rotationToQuaternion(m);
        // m.get(q); EAK: there's a bug in java8 Matrix4d.get(Quat4d) that results in occasional NaNs after sqrt of a negative
        //q.set(m);
        return new Pose(new Point(v.x, v.y, v.z), new Quaternion(q.x, q.y, q.z, q.w));
    }

    public static TransformStamped convertToTransformStamped(Matrix4d m) {
        Vector3d v = new Vector3d();
        m.get(v);
        Quat4d q = rotationToQuaternion(m);
        // m.get(q); EAK: there's a bug in java8 Matrix4d.get(Quat4d) that results in occasional NaNs after sqrt of a negative
        //q.set(m);
        return convertToTransformStamped(v,q);
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
    public static double[] quaternionToEuler(Quaternion q) {
        double[] angles = {0, 0, 0};

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.getW() * q.getX() + q.getY() * q.getZ());
        double cosr_cosp = 1 - 2 * (q.getX() * q.getX() + q.getY() * q.getY());
        angles[ROLL] = Math.atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.getW() * q.getY() - q.getZ() * q.getX());
        if (Math.abs(sinp) >= 1) {
            angles[PITCH] = Math.copySign(Math.PI / 2, sinp); // use 90 degrees if out of range
        } else {
            angles[PITCH] = Math.asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.getW() * q.getZ() + q.getX() * q.getY());
        double cosy_cosp = 1 - 2 * (q.getY() * q.getY() + q.getZ() * q.getZ());
        angles[YAW] = Math.atan2(siny_cosp, cosy_cosp);

        return angles;
    }

    public static Quaternion eulerToQuaternion(double yaw, double pitch, double roll) {
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        Quaternion q = new Quaternion();
        q.setW(cr * cp * cy + sr * sp * sy);
        q.setX(sr * cp * cy - cr * sp * sy);
        q.setY(cr * sp * cy + sr * cp * sy);
        q.setZ(cr * cp * sy - sr * sp * cy);

        return q;
    }

    public static Quat4d rotationToQuaternion(Matrix4d T) {
        double[][] R = new double[3][3];
        R[0][0] = T.getM00();
        R[0][1] = T.getM01();
        R[0][2] = T.getM02();
        R[1][0] = T.getM10();
        R[1][1] = T.getM11();
        R[1][2] = T.getM12();
        R[2][0] = T.getM20();
        R[2][1] = T.getM21();
        R[2][2] = T.getM22();

        double[] quat = rotationToQuaternion(R);
        return new Quat4d(quat[1], quat[2], quat[3], quat[0]);
    }

    /**
     * Shepperd's method for converting a rotation matrix to a quaternion.
     *
     * @param R - 3x3 rotation matrix
     * @return - quaternion
     */
    public static double[] rotationToQuaternion(double[][] R) {
        double w, x, y, z;

        double trace = R[0][0] + R[1][1] + R[2][2];

        if (trace > 0) {
            double s = 0.5 / Math.sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (R[2][1] - R[1][2]) * s;
            y = (R[0][2] - R[2][0]) * s;
            z = (R[1][0] - R[0][1]) * s;
        } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
            double s = 2.0 * Math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
            w = (R[2][1] - R[1][2]) / s;
            x = 0.25 * s;
            y = (R[0][1] + R[1][0]) / s;
            z = (R[0][2] + R[2][0]) / s;
        } else if (R[1][1] > R[2][2]) {
            double s = 2.0 * Math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
            w = (R[0][2] - R[2][0]) / s;
            x = (R[0][1] + R[1][0]) / s;
            y = 0.25 * s;
            z = (R[1][2] + R[2][1]) / s;
        } else {
            double s = 2.0 * Math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
            w = (R[1][0] - R[0][1]) / s;
            x = (R[0][2] + R[2][0]) / s;
            y = (R[1][2] + R[2][1]) / s;
            z = 0.25 * s;
        }

        return new double[]{w, x, y, z};
    }
}
