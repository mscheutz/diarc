/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.util;

import edu.tufts.hrilab.action.db.Database;

import java.util.*;

public class SymbolResolver {

  private static Map<String, String> gridToPoly = new HashMap<>();
  private static Map<String, String> polyToGrid = new HashMap<>();
  private static Map<String, String> polyToAction = new HashMap<>();

  static {
    polyToAction.put("BREAK_BLOCK", "break_and_pickup");
    polyToAction.put("COLLECT", "collect_from");
    polyToAction.put("INTERACT", "interact_with");
  }

  static {
    gridToPoly.put("air", "minecraft:air");
    gridToPoly.put("crafting_table", "minecraft:crafting_table");
    gridToPoly.put("plank", "minecraft:planks");
    gridToPoly.put("pogo_stick", "polycraft:wooden_pogo_stick");
    gridToPoly.put("rubber", "polycraft:sack_polyisoprene_pellets");
    gridToPoly.put("stick", "minecraft:stick");
    gridToPoly.put("log", "minecraft:log");
    gridToPoly.put("tree_tap", "polycraft:tree_tap");
    gridToPoly.put("wall", "minecraft:bedrock");
    // phase 2 types
    gridToPoly.put("safe", "polycraft:safe");
    gridToPoly.put("key", "polycraft:key");
    gridToPoly.put("iron_pickaxe", "minecraft:iron_pickaxe");
    gridToPoly.put("trader1", "polycraft:trader1");
    gridToPoly.put("trader2", "polycraft:trader2");
    gridToPoly.put("block_of_diamond", "minecraft:diamond_block");
    gridToPoly.put("block_of_platinum", "polycraft:block_of_platinum");
    gridToPoly.put("block_of_titanium", "polycraft:block_of_titanium");
    gridToPoly.put("chest", "polycraft:plastic_chest");
    gridToPoly.put("diamond_ore", "minecraft:diamond_ore");
    gridToPoly.put("diamond", "minecraft:diamond");
    gridToPoly.put("unlocked_safe", "polycraft:unlocked_safe");
    gridToPoly.put("door", "minecraft:wooden_door");
    gridToPoly.put("sapling", "minecraft:sapling");

    for (Map.Entry<String, String> entry : gridToPoly.entrySet()) {
      polyToGrid.put(entry.getValue(), entry.getKey());
    }
  }

  public static String toAction(String name) {
    String action = polyToAction.get(name);
    if (action == null) {
      action = name.toLowerCase();
    }
    if (Database.getActionDB().actionExists(action)) {
      return action;
    }
    return null;
  }

  public static String toAction(String name, String arg) {
    String action = polyToAction.get(name);
    if (action==null) {
      action = name.toLowerCase();
    }
    if (Database.getActionDB().actionExists(action+"_"+arg)) {
      return action+"_"+arg;
    } else {
      return toAction(name);
    }
  }

  public static String toGridItem(String name) {
    return polyToGrid.getOrDefault(name, name.replace(":", "_"));
  }

  public static String toGridItem(String name, String variant) {
    if (variant == null || variant.isEmpty()) {
      return polyToGrid.getOrDefault(name, name.replace(":", "_"));
    } else {
      return variant + "_" + polyToGrid.getOrDefault(name, name.replace(":", "_"));
    }
  }

  public static String toPolyItem(String name) {
    String r = gridToPoly.get(name);
    if (r == null) {
      if (name.contains(":")) {
        r = name;
      } else if (name.startsWith("minecraft") || name.startsWith("polycraft")) {
        r = name.replaceFirst("_", ":");
      } else if (name.contains("_")) {
        int index = name.indexOf('_');
        String tmp = name.substring(index+1);
        r = gridToPoly.get(tmp);
        if (r==null) {
          r = tmp;
        }
      } else {
        r = name; // pass back what was passed in
      }
    }
    return r;
  }

  public static String[] toGridPos(String[] pos) {
    if (pos.length == 2) {
      return new String[]{pos[1], pos[0]};
    } else if (pos.length == 3) {
      return new String[]{pos[0], pos[2]};
    } else {
      return null;
    }
  }

  public static String[] toGridPos(int[] pos) {
    if (pos.length == 2) {
      return new String[]{String.valueOf(pos[1]), String.valueOf(pos[0])};
    } else if (pos.length == 3) {
      return new String[]{String.valueOf(pos[0]), String.valueOf(pos[2])};
    } else {
      return null;
    }
  }

  public static String toPolycraftLocation(String name) {
    String[] delimited = name.split("_");
    return delimited[0] + ",17," + delimited[1];
  }

}
