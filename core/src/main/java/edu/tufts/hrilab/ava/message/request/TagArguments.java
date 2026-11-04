/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.ava.message.request;

import java.util.Map;

public class TagArguments {
  public double x;
  public double y;
  public double theta;
  public String name;
  public Map<String, String> attributes;

  public TagArguments(String name, double x, double y, double theta, Map<String, String> attributes) {
    this.name = name;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.attributes = attributes;
  }
}
