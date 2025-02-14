/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;


public class MapMetadataResponse {
  public String name;
  public String format;
  public long lastModified;
  public double mapScale;
  public double mapScaleX;
  public double mapScaleY;
  public int width;
  public int height;
  public int[][] scales;
  public String company;
  public String building;
  public String floor;
}