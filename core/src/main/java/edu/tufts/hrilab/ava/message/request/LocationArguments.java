/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.request;

/*"waypoint": {
  "id": 1477
  },
  "map": "Building10-2_11_26_14",
  "room": "CR-Mach5"
  }*/

public class LocationArguments {
  public WaypointArguments waypoint;
  public String map;
  public String room;

  public LocationArguments(String mapString, String roomString ) {
    map = mapString;
    room = roomString;
  }
  public LocationArguments(String mapString, String roomString, int waypointId ) {
    waypoint = new WaypointArguments(waypointId);
    map = mapString;
    room = roomString;
  }
}
