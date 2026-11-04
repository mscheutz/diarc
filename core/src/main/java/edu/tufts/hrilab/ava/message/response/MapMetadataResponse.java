/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;


import java.io.Serializable;

public class MapMetadataResponse implements Serializable {
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
  public double originX;
  public double originY;
}