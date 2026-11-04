/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

public class RMSDestinationPose {
  public double x;
  public double y;
  public double theta;

  public RMSDestinationPose(double x, double y, double theta ) {
    this.x = x;
    this.y = y;
    this.theta = theta;
  }
}
