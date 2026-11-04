/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;


public class PointResponse {
  public int id;
  public String name;
  public LocationResponse location;

  public PointResponse(int idInt, String nameString, LocationResponse locationObject) {
    id = idInt;
    name = nameString;
    location = locationObject;
  }
}
