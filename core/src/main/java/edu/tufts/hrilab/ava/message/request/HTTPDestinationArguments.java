/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

//HTTP server and RMS server have different requirements for Destination arguments
public class HTTPDestinationArguments {
    public String mapId;
    public float x;
    public float y;
    public float theta;

    public HTTPDestinationArguments ( String mapId, float x, float y, float theta ) {
        this.mapId = mapId;
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}