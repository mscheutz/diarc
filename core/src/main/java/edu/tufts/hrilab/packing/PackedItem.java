package edu.tufts.hrilab.packing;

import java.awt.*;

// represents a single packed item in 3d space.
public class PackedItem {
  final Rectangle rect; // 2d position in the layer (x, y, width, length)
  final int layer;      // the layer number based on the item height
  final double centerX, centerY, centerZ;
  final int itemH;

  PackedItem(Rectangle rect, int layer, int itemH) {
    this.rect = rect;
    this.layer = layer;
    this.itemH = itemH;
    // calculate the center of the item relative to the pallet origin (0,0,0)
    this.centerX = rect.x + rect.width / 2.0;
    this.centerY = rect.y + rect.height / 2.0;
    this.centerZ = (layer - 1) * itemH + itemH; // the Z value is on the surface of the item
  }

  @Override
  public String toString() {
    return String.format("Layer: %d, Center: (%.1f, %.1f, %.1f), Orientation: %d x %d",
            layer, centerX, centerY, centerZ, rect.width, rect.height);
  }
}