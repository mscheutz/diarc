package edu.tufts.hrilab.packing;

import java.awt.*;

// represents a single packed item in 3d space.
public class PackedItem {
  final Rectangle rect; // 2d position in the layer (x, y, width, length)
  final int layer;      // the layer number based on the item height
  final float centerX, centerY, centerZ;
  final int itemH;
  private float angle;

  PackedItem(Rectangle rect, int layer, int itemH) {
    this.rect = rect;
    this.layer = layer;
    this.itemH = itemH;
    // calculate the center of the item relative to the crate origin (0,0,0)
    this.centerX = (float) (rect.x + rect.width / 2.0);
    this.centerY = (float) (rect.y + rect.height / 2.0);
    this.centerZ = (layer - 1) * itemH + itemH; // the Z value is on the surface of the item
    this.angle = 0.0F;
    if (rect.width > rect.height) {
      this.angle = (float) (-90 * (Math.PI / 180));
    }
  }

  public float getCenterX() {
    return centerX;
  }

  public float getCenterY() {
    return centerY;
  }

  public float getCenterZ() {
    return centerZ;
  }

  public float getAngle() {
    return angle;
  }

  @Override
  public String toString() {
    return String.format("Layer: %d, Center: (%.1f, %.1f, %.1f), Orientation: %d x %d",
            layer, centerX, centerY, centerZ, rect.width, rect.height);
  }
}