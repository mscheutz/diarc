/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

public class MapResponse {
  public WaypointResponse[] waypoints;
  public RegionResponse[] regions;
  //public RMSPathResponse[] paths;
  public PointResponse[] markers;
}
