/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs.gson;

/**
 * MPoseGson is a helper class to marshal JSON representations of a pose into
 */
public class MPoseGson {

    //Values in mm
    public float x;
    public float y;
    public float z;
    //rotation in rads around x axis
    public float a;
    //rotation in rads around y axis
    public float b;
    //rotation in rads around z axis
    public float c;

    /**
     * A blank constructor allows for the creation of a java class to contain our JSON
     */
    public MPoseGson() {

    }
}
