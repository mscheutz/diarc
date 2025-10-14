package edu.tufts.hrilab.packing;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * An implementation of crate packing with homogeneous items of
 * the same length, width and height (l,w,h) which are placed in a box
 * such that items are layered in layers of h with (l,w) either
 * horizontally or vertically placed to maxmize 2d packing in each layer;
 * all layers are repeated
 */
public class CratePackingComponent extends DiarcComponent {

  //TODO:Likely will want to modify this to not have 1:1 relationship with a packing instance?
  int crateL, crateW, crateH, packedItemL, packedItemW, packedItemH;
  PackingResult3D cachedResult = null;
  int itemIndex = 0;

  public CratePackingComponent() {
    crateL = -1;
    crateW = -1;
    crateH = -1;
    packedItemL = -1;
    packedItemW = -1;
    packedItemH = -1;
  }

  public CratePackingComponent(int crateL, int crateW, int crateH, int packedItemL, int packedItemW, int packedItemH) {
    this.crateL = crateL;
    this.crateW = crateW;
    this.crateH = crateH;
    this.packedItemL = packedItemL;
    this.packedItemW = packedItemW;
    this.packedItemH = packedItemH;
  }

  @Override
  public void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("crateL") || cmdLine.hasOption("crateW") || cmdLine.hasOption("crateH")) {
      if (cmdLine.hasOption("crateL") && cmdLine.hasOption("crateW") && cmdLine.hasOption("crateH")) {
        crateL = Integer.parseInt(cmdLine.getOptionValue("crateL"));
        crateW = Integer.parseInt(cmdLine.getOptionValue("crateW"));
        crateH = Integer.parseInt(cmdLine.getOptionValue("crateH"));
      } else {
        log.warn("[parseArgs] Not all crate dimensions supplied, ignoring");
      }
    }

    if (cmdLine.hasOption("itemL") || cmdLine.hasOption("itemW") || cmdLine.hasOption("itemH")) {
      if (cmdLine.hasOption("itemL") && cmdLine.hasOption("itemW") && cmdLine.hasOption("itemH")) {
        packedItemL = Integer.parseInt(cmdLine.getOptionValue("itemL"));
        packedItemW = Integer.parseInt(cmdLine.getOptionValue("itemW"));
        packedItemH = Integer.parseInt(cmdLine.getOptionValue("itemH"));
      } else {
        log.warn("[parseArgs] Not all item dimensions supplied, ignoring");
      }
    }

    //if (crateL != -1 && packedItemL != -1) {
    //  solveCratePacking3D();
    //}
  }

  @Override
  public List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("crateL").hasArg().build());
    options.add(Option.builder("crateW").hasArg().build());
    options.add(Option.builder("crateH").hasArg().build());
    options.add(Option.builder("itemL").hasArg().build());
    options.add(Option.builder("itemW").hasArg().build());
    options.add(Option.builder("itemH").hasArg().build());
    return options;
  }

  /**
   * the 3d solver which calculates the packing based on a fixed item
   * height, allowing only for 90-degree horizontal rotation of the item
   */
  @TRADEService
  public PackingResult3D solveCratePacking3D() {
    if (crateL == -1 || crateW == -1 || crateH == -1 || packedItemL == -1 || packedItemW == -1 || packedItemH == -1) {
      log.error("[solveCratePacking3D] crate and item dimensions must be fully set before running solve. Current values {},{},{} {},{},{}", crateL, crateW, crateH, packedItemL, packedItemW, packedItemH);
      return null;
    }
    return solveCratePacking3D(crateL, crateW, crateH, packedItemL, packedItemW, packedItemH);
  }
  @TRADEService
  public PackingResult3D solveCratePacking3D(int crateLength, int crateW, int crateH, int rectL, int rectW, int rectH) {
    itemIndex = 0;
    if (crateH < rectH) {
      cachedResult = new PackingResult3D(0, 0, new ArrayList<>(), new PackingResult(0, new ArrayList<>()), "None");
      return cachedResult;
    }

    int numLayers = crateH / rectH;
    PackingResult layerResult = solve(crateLength, crateW, rectL, rectW);
    int totalCount = numLayers * layerResult.count;

    List<PackedItem> allItems = new ArrayList<>();
    if (totalCount > 0) {
      for (int i = 1; i <= numLayers; i++) {
        for (Rectangle r : layerResult.rectangles) {
          allItems.add(new PackedItem(r, i, rectH));
        }
      }
    }

    String orientation = String.format("l x w footprint (%d x %d), fixed height %d", rectL, rectW, rectH);
    cachedResult = new PackingResult3D(totalCount, numLayers, allItems, layerResult, orientation);
    return cachedResult;
  }

  /**
   * the 2d solver solves for a crate with L >= W and then
   * transposes the result if the original dimensions were swapped
   */
  private PackingResult solve(int crateLength, int crateW, int rectL, int rectW) {
    boolean swapped = false;
    if (crateLength < crateW) {
      int temp = crateLength;
      crateLength = crateW;
      crateW = temp;
      swapped = true;
    }

    Map<String, PackingResult> memo = new HashMap<>();
    PackingResult canonicalResult = pack(crateLength, crateW, rectL, rectW, memo);

    if (swapped) {
      List<Rectangle> transposedRects = new ArrayList<>();
      for (Rectangle r : canonicalResult.rectangles) {
        transposedRects.add(new Rectangle(r.y, r.x, r.height, r.width));
      }
      return new PackingResult(canonicalResult.count, transposedRects);
    } else {
      return canonicalResult;
    }
  }

  /**
   * the core recursive method for 2d packing
   */
  private PackingResult pack(int currentL, int currentW, int l, int w, Map<String, PackingResult> memo) {
    String key = currentL + "x" + currentW;
    if (memo.containsKey(key)) {
      return memo.get(key);
    }

    PackingResult result1 = solveForFixedOrientation(currentL, currentW, l, w, l, w, memo);
    PackingResult bestResult = result1;

    if (l != w) {
      PackingResult result2 = solveForFixedOrientation(currentL, currentW, w, l, l, w, memo);
      if (result2.count > bestResult.count) {
        bestResult = result2;
      }
    }

    memo.put(key, bestResult);
    return bestResult;
  }

  /**
   * helper for solving packing with a fixed rectangle orientation within each of five regions
   */
  private PackingResult solveForFixedOrientation(int currentL, int currentW, int rectL, int rectW, int origL, int origW, Map<String, PackingResult> memo) {
    if (currentL < rectL || currentW < rectW) {
      return new PackingResult(0, new ArrayList<>());
    }

    int baseCount = (currentL / rectL) * (currentW / rectW);
    List<Rectangle> baseRects = new ArrayList<>();
    for (int i = 0; i < currentL / rectL; i++) {
      for (int j = 0; j < currentW / rectW; j++) {
        baseRects.add(new Rectangle(i * rectL, j * rectW, rectL, rectW));
      }
    }
    PackingResult maxResult = new PackingResult(baseCount, baseRects);

    for (int cutW = 1; cutW < currentW; cutW++) {
      PackingResult topAreaResult = pack(currentL, cutW, origL, origW, memo);
      PackingResult bottomAreaResult = pack(currentL, currentW - cutW, origL, origW, memo);
      int totalCount = topAreaResult.count + bottomAreaResult.count;
      if (totalCount > maxResult.count) {
        List<Rectangle> combinedRects = new ArrayList<>(topAreaResult.rectangles);
        for (Rectangle r : bottomAreaResult.rectangles) {
          combinedRects.add(new Rectangle(r.x, r.y + cutW, r.width, r.height));
        }
        maxResult = new PackingResult(totalCount, combinedRects);
      }
    }

    for (int cutL = 1; cutL < currentL; cutL++) {
      PackingResult leftAreaResult = pack(cutL, currentW, origL, origW, memo);
      PackingResult rightAreaResult = pack(currentL - cutL, currentW, origL, origW, memo);
      int totalCount = leftAreaResult.count + rightAreaResult.count;
      if (totalCount > maxResult.count) {
        List<Rectangle> combinedRects = new ArrayList<>(leftAreaResult.rectangles);
        for (Rectangle r : rightAreaResult.rectangles) {
          combinedRects.add(new Rectangle(r.x + cutL, r.y, r.width, r.height));
        }
        maxResult = new PackingResult(totalCount, combinedRects);
      }
    }

    return maxResult;
  }

  @TRADEService
  public void setCrateLength(int crateLength) {
    this.crateL = crateLength;
  }
  @TRADEService
  public void setCrateWidth(int crateW) {
    this.crateW = crateW;
  }
  @TRADEService
  public void setCrateHeight(int crateH) {
    this.crateH = crateH;
  }
  @TRADEService
  public void setCrateDimensions(int crateLength, int crateW, int crateH) {
    this.crateL = crateLength;
    this.crateW = crateW;
    this.crateH = crateH;
  }
  @TRADEService
  public void setPackedItemLength(int itemL) {
    packedItemL = itemL;
  }
  @TRADEService
  public void setPackedItemWidth(int itemW) {
    packedItemW = itemW;
  }
  @TRADEService
  public void setPackedItemHeight(int itemH) {
    packedItemH = itemH;
  }
  @TRADEService
  public void setPackedItemDimensions(int itemL, int itemW, int itemH) {
    packedItemL = itemL;
    packedItemW = itemW;
    packedItemH = itemH;
  }

  @TRADEService
  public PackingResult3D getCachedPackingResult3D() {
    if (cachedResult == null) {
      log.warn("[getCachedPackingResult3D]  No cached packing result found, run solveCratePacking3D first");
      return null;
    }
    return cachedResult;
  }

  @TRADEService
  public void resetPackedItemIndex() {
    itemIndex = 0;
  }

  @TRADEService
  public PackedItem getNextPackedItem() {
    if (cachedResult == null) {
      log.warn("[getNextPackedItem] No cached packing result found, run solveCratePacking3D first");
      return null;
    }
    return cachedResult.allItems.get(itemIndex++);
  }

  @TRADEService
  public int getNumLayers() {
    if (cachedResult == null) {
      log.warn("[getNumLayers] No cached packing result found, run solveCratePacking3D first");
      return -1;
    }
    return cachedResult.numLayers;
  }

  @TRADEService
  public int getTotalNumberOfItems() {
    if (cachedResult == null) {
      log.warn("[getTotalNumberOfItems] No cached packing result found, run solveCratePacking3D first");
      return -1;
    }
    return cachedResult.totalCount;
  }

  @TRADEService
  public int getNumberOfItemsPerLayer() {
    if (cachedResult == null) {
      log.warn("[getNumberOfItemsPerLayer] No cached packing result found, run solveCratePacking3D first");
      return -1;
    }
    return cachedResult.layerPacking.count;
  }

  @TRADEService
  public PackingResult getLayerRectangles() {
    if (cachedResult == null) {
      log.warn("[getLayerRectangle] No cached packing result found, run solveCratePacking3D first");
      return null;
    }
    return cachedResult.layerPacking;
  }

  @TRADEService
  public int getCrateLength() {
    return crateL;
  }

  @TRADEService
  public int getCrateWidth() {
    return crateW;
  }

  @TRADEService
  public int getCrateHeight() {
    return crateH;
  }

  @TRADEService
  public int getPackedItemLength() {
    return packedItemL;
  }

  @TRADEService
  public int getPackedItemWidth() {
    return packedItemW;
  }

  @TRADEService
  public int getPackedItemHeight() {
    return packedItemH;
  }
}