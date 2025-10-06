package edu.tufts.hrilab.packing;

import java.util.List;

// the 3d packing result with the list of all packed items
public class PackingResult3D {
  final int totalCount;
  final int numLayers;
  final List<PackedItem> allItems;
  final PackingResult layerPacking; // Keep for visualization
  final String orientation;

  PackingResult3D(int totalCount, int numLayers, List<PackedItem> allItems, PackingResult layerPacking, String orientation) {
    this.totalCount = totalCount;
    this.numLayers = numLayers;
    this.allItems = allItems;
    this.layerPacking = layerPacking;
    this.orientation = orientation;
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    for (PackedItem item : allItems) {
      sb.append(item.toString());
      sb.append("\n");
    }
    return sb.toString();
  }
}