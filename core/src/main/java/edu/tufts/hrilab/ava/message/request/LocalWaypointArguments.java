/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

public class LocalWaypointArguments {
    public float x;
    public float y;
    public float theta;

    public LocalWaypointArguments(float xFloat, float yFloat, float thetaFloat ) {
        x = xFloat;
        y = yFloat;
        theta = thetaFloat;
    }
}