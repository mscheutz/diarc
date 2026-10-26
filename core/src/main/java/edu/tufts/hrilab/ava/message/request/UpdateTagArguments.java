/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.ava.message.request;

public class UpdateTagArguments {
  public int id;
  public Pose pose;

  public UpdateTagArguments(int id, double x, double y, double theta) {
    this.id = id;
    this.pose = new Pose(x, y, theta);
  }

  private static class Pose {
    public double x;
    public double y;
    public double theta;

    public Pose(double x, double y, double theta) {
      this.x = x;
      this.y = y;
      this.theta = theta;
    }
  }
}
