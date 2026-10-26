/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

public class RegionResponse {
  public int id;
  public String name;
  public RMSCornerResponse[] corners;
  public PointResponse location;
}
