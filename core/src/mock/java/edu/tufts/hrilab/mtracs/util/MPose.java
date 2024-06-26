/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs.util;

import edu.tufts.hrilab.mtracs.gson.MPoseGson;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.io.Serializable;

/**
 * MPose represents one robot End Effector pose, including all the information required for pose representation on
 * an CR800 controller, as well as associated metadata for TRACS.
 */
public class MPose implements Serializable {
    //Distance for autogenerated approach poses in mm
    private final float approachOffset = 170.0f;

    @Override
    public String toString() {
        return "MPose{" +
                "approachOffset=" + approachOffset +
                ", surfaceHeight=" + surfaceHeight +
                ", x=" + x +
                ", y=" + y +
                ", z=" + z +
                ", a=" + a +
                ", b=" + b +
                ", c=" + c +
                ", l1=" + l1 +
                ", l2=" + l2 +
                ", s1=" + s1 +
                ", s2=" + s2 +
                '}';
    }

    //DIARC values
    private float surfaceHeight = 0f;

    //Values in mm
    protected float x;
    protected float y;
    protected float z;
    //rotation in rads around x axis
    protected float a;
    //rotation in rads around y axis
    protected float b;
    //rotation in rads around z axis
    protected float c;

    protected float l1;
    protected float l2;

    protected long s1;
    protected long s2;

    /**
     * Constructs an MPose from the given parameters.
     *
     * @param x The x value of the pose in millimeters.
     * @param y The y value of the pose in millimeters.
     * @param z The z value of the pose in millimeters.
     * @param a The a rotation of the pose in radians.
     * @param b The b rotation of the pose in radians.
     * @param c The c rotation of the pose in radians.
     */
    public MPose(float x, float y, float z, float a, float b, float c) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.a = a;
        this.b = b;
        this.c = c;
        //Todo: Will: I don't fully understand what this code stands for, but it seems to be the default for all cr800 components
        //We should make sure that this doesn't need to be configurable though, because we need it to be the same for every
        //MPose we send the robot to it seems
        this.s1 = 7;
        this.s2 = 0;
    }

    /**
     * Constructs an empty MPose.
     */
    public MPose() {
        this(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    /**
     * Constructs an MPose as a shallow copy of the given MPose.
     *
     * @param other The MPose to construct a copy of.
     */
    public MPose(MPose other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        this.a = other.a;
        this.b = other.b;
        this.c = other.c;
        this.l1 = other.l1;
        this.l2 = other.l2;
        this.s1 = other.s1;
        this.s2 = other.s2;
    }

    /**
     * Constructs an MPose as a shallow copy of the given MPoseGson.
     *
     * @param other The MPose to construct a copy of.
     */
    public MPose(MPoseGson other) {
        this(other.x, other.y, other.z, other.a, other.b, other.c);
    }




    /**
     * Adjusts <code>this</code> MPose by vector adding it to the given MPose
     *
     * @param other The pose values to add to this MPose as a MPose
     */
    public void add(MPose other) {
        this.x += other.x;
        this.y += other.y;
        this.z += other.z;
        this.a += other.a;
        this.b += other.b;
        this.c += other.c;
    }

    /**
     * Gets the just the representation of this MPose's translation in meters
     *
     * @return The Point3d representation of this pose's translation
     */
    public Point3d getTranslation() {
        return new Point3d(x / 1000.0, y / 1000.0, z / 1000.0);
    }

    /**
     * Gets the just the representation of this MPose's orientation in radians (ABC format)
     *
     * @return The Point3d representation of this pose's orientation
     */
    public Point3d getOrientation() {
        return new Point3d(a, b, c);
    }

    /**
     * Sets the just the representation of this MPose's translation in meters
     *
     * @param t The translation to set as a Point3d
     */
    public void setTranslation(Point3d t) {
        setXMeters((float)t.x);
        setYMeters((float)t.y);
        setZMeters((float)t.z);
    }

    /**
     * Sets the just the representation of this MPose's orientation in radians (ABC format)
     *
     * @param o The orientation to set as a Point3d
     */
    public void setOrientation(Point3d o) {
        setA((float)o.x);
        setB((float)o.y);
        setC((float)o.z);
    }

    /**
     * Sets the just the representation of this MPose's orientation as a quaternion
     *
     * @param q The orientation to set as a Quat4d
     */
    public void setOrientation(Quat4d q) {
        //Todo: Will: this code is copied from Convert.java in moveitutil - should that be reorganized to be more general across diarc and imported in the jar?
        // quat4d -> YZX Euler Angles (NASA Standard Aeroplane) (https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/indexLocal.htm)
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
        Point3d rawAngles = new Point3d(bank, heading, attitude);

    }

    /**
     * Fetches the x value of this pose.
     *
     * @return The x value of this pose in millimeters.
     */
    public float getX() {
        return x;
    }

    /**
     * Fetches the x value of this pose.
     *
     * @return The x value of this pose in meters.
     */
    public float getXMeters() {
        return x / 1000.0f;
    }

    /**
     * Sets the x value of this pose.
     *
     * @param x The x value to set in millimeters.
     */
    public void setX(float x) {
        this.x = x;
    }

    /**
     * Sets the x value of this pose.
     *
     * @param x The x value to set in meters.
     */
    public void setXMeters(float x) {
        this.x = x * 1000.0f;
    }

    /**
     * Fetches the y value of this pose.
     *
     * @return The y value of this pose in millimeters.
     */
    public float getY() {
        return y;
    }

    /**
     * Fetches the y value of this pose.
     *
     * @return The y value of this pose in meters.
     */
    public float getYMeters() {
        return y / 1000.0f;
    }

    /**
     * Sets the y value of this pose.
     *
     * @param y The y value to set in millimeters.
     */
    public void setY(float y) {
        this.y = y;
    }

    /**
     * Sets the y value of this pose.
     *
     * @param y The y value to set in meters.
     */
    public void setYMeters(float y) {
        this.y = y * 1000.0f;
    }

    /**
     * Fetches the z value of this pose.
     *
     * @return The z value of this pose in millimeters.
     */
    public float getZ() {
        return z;
    }

    /**
     * Fetches the z value of this pose.
     *
     * @return The z value of this pose in meters.
     */
    public float getZMeters() {
        return z / 1000.0f;
    }

    /**
     * Sets the z value of this pose.
     *
     * @param z The z value to set in millimeters.
     */
    public void setZ(float z) {
        this.z = z;
    }

    /**
     * Sets the z value of this pose.
     *
     * @param z The z value to set in meters.
     */
    public void setZMeters(float z) {
        this.z = z * 1000.0f;
    }

    /**
     * Gets the a rotation of this pose.
     *
     * @return The a rotation of this pose in radians.
     */
    public float getA() {
        return a;
    }

    /**
     * Sets the a rotation of this pose.
     *
     * @param a The a rotation to set in radians.
     */
    public void setA(float a) {
        this.a = a;
    }

    /**
     * Gets the b rotation of this pose.
     *
     * @return The b rotation of this pose in radians.
     */
    public float getB() {
        return b;
    }

    /**
     * Sets the b rotation of this pose.
     *
     * @param b The b rotation to set in radians.
     */
    public void setB(float b) {
        this.b = b;
    }

    /**
     * Gets the c rotation of this pose.
     *
     * @return The c rotation of this pose in radians.
     */
    public float getC() {
        return c;
    }

    /**
     * Sets the c rotation of this pose.
     *
     * @param c The c rotation to set in radians.
     */
    public void setC(float c) {
        this.c = c;
    }

    /**
     * Gets the L1 value of this pose.
     *
     * @return The L1 value of this pose as a float.
     */
    public float getL1() {
        return l1;
    }

    /**
     * Sets the L1 value of this pose.
     *
     * @param l1 The L1 value to set as a float.
     */
    public void setL1(float l1) {
        this.l1 = l1;
    }

    /**
     * Gets the L2 value of this pose.
     *
     * @return The L2 value of this pose as a float.
     */
    public float getL2() {
        return l2;
    }

    /**
     * Sets the L2 value of this pose.
     *
     * @param l2 The L2 value to set as a float.
     */
    public void setL2(float l2) {
        this.l2 = l2;
    }

    /**
     * Gets the S1 value of this pose.
     *
     * @return The S1 value of this pose as a float.
     */
    public long getS1() {
        return s1;
    }

    /**
     * Sets the S1 value of this pose.
     *
     * @param s1 The S1 value to set as a float.
     */
    public void setS1(long s1) {
        this.s1 = s1;
    }

    /**
     * Gets the S2 value of this pose.
     *
     * @return The S2 value of this pose as a float.
     */
    public long getS2() {
        return s2;
    }

    /**
     * Sets the S2 value of this pose.
     *
     * @param s2 The S2 value to set as a float.
     */
    public void setS2(long s2) {
        this.s2 = s2;
    }

    /**
     * Gets the surface height associated with this pose.
     *
     * @return The surface height associated with this pose in millimeters.
     */
    public float getSurfaceHeight() {
        return surfaceHeight;
    }

    /**
     * Sets the surface height associated with this pose.
     *
     * @param surfaceHeight The surface height to be associated with this pose in millimeters.
     */
    public void setSurfaceHeight(float surfaceHeight) {
        this.surfaceHeight = surfaceHeight;
    }

    public MPose getVerticalApproachPose(){
        MPose approach = new MPose(this);
        approach.setZ(approach.getZ() + approachOffset);
        return approach;
    }

    public MPose getHorizontalApproachPose(){
        MPose approach = new MPose(this);
        approach.setX(approach.getX() + approachOffset);
        return approach;
    }
}
