/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.request;

public class MapArguments {
  public String name;
  public String type = "all";

  public MapArguments(String map ) {
    name = map;
  }
}