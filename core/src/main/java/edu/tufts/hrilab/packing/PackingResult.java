package edu.tufts.hrilab.packing;

import java.awt.*;
import java.util.List;

// the 2d packing result containing the number and list of placed items
public class PackingResult {
  final int count;
  final java.util.List<Rectangle> rectangles;

  PackingResult(int count, List<Rectangle> rectangles) {
    this.count = count;
    this.rectangles = rectangles;
  }
}
