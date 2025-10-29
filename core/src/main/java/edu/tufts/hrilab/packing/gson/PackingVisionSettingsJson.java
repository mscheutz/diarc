/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing.gson;

public class PackingVisionSettingsJson {

  public int lowThreshold;
  public int highThreshold;
  public int dilationIterations;
  public double alpha;
  public int selectedCamera;

  public PackingVisionSettingsJson(int lowThreshold, int highThreshold, int dilationIterations, double alpha, int selectedCamera) {
    this.lowThreshold = lowThreshold;
    this.highThreshold = highThreshold;
    this.dilationIterations = dilationIterations;
    this.alpha = alpha;
    this.selectedCamera = selectedCamera;
  }
}

