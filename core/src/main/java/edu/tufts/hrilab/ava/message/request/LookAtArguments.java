/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

public class LookAtArguments {
  public double x;
  public double y;
  public double z;
  public double duration;

  public LookAtArguments ( double x, double y, double z, double duration ) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.duration = duration;
  }
}