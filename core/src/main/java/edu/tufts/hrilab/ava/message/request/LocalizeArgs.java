/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

public class LocalizeArgs {
  private int mapId;
  private Position position;
  private Double theta;

  public LocalizeArgs(int mapId, double x, double y) {
    this.mapId = mapId;
    this.position = new Position(x, y);
    this.theta = null;
  }

  public LocalizeArgs(int mapId, double x, double y, double theta) {
    this.mapId = mapId;
    this.position = new Position(x, y);
    this.theta = theta;
  }

  public LocalizeArgs(int mapId) {
    this.mapId = mapId;
    this.position = null;
    this.theta = null;
  }

  private static class Position {
    public double x;
    public double y;

    public Position(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }
}
