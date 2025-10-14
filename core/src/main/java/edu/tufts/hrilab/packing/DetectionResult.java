/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

import java.util.List;

public class DetectionResult {
  public final List<Vec2> corners;
  public final List<RectangleDetection.HoughLine> lines;
  DetectionResult(List<Vec2> corners, List<RectangleDetection.HoughLine> lines) {
    this.corners = corners;
    this.lines = lines;
  }
}
