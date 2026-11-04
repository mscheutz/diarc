/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

/*{
  "id": 1477
  }*/

public class WaypointResponse {
  public String name;
  public PointResponse[] points;

  public WaypointResponse(String nameString ) {
    name = nameString;
  }
}
