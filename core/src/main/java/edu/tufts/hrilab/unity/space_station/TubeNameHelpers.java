/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;

import java.util.ArrayList;
import java.util.List;

/**
 * Helper methods to convert between various space station tube name forms.
 */
public class TubeNameHelpers {

  public static String toSpoken(String str) {
    return String.join(" ", str.split("(?<=[a-z])(?=[A-Z])"));
  }

  public static String capitalize(String str) {
    return str.substring(0, 1).toUpperCase() + str.substring(1);
  }

  public static Symbol toFullTube(Symbol area, Symbol side, Symbol number) {
    return Factory.createSymbol(area.toString() + capitalize(side.toString()) + capitalize(number.toString()));
  }

  public static Symbol toTubeNumber(Symbol side, Symbol number) {
    return Factory.createSymbol(capitalize(side.toString()) + capitalize(number.toString()));
  }

  public static Symbol getTubeSide(Symbol tubeCombined) {
    String[] parts = tubeCombined.toString().split("(?=[A-Z])");
    return Factory.createSymbol(parts[0].toLowerCase());
  }

  public static Symbol getTubeNumber(Symbol tubeCombined) {
    String[] parts = tubeCombined.toString().split("(?=[A-Z])");
    return Factory.createSymbol(parts[1].toLowerCase());
  }

  public static List<Symbol> fromFullTube(Symbol fullTube) {
    List<Symbol> symbolList = new ArrayList<>();
    for (String str : fullTube.toString().split("(?<!(^|[A-Z]))(?=[A-Z])|(?<!^)(?=[A-Z][a-z])")) {
      symbolList.add(Factory.createSymbol(str.toLowerCase()));
    }
    return symbolList;
  }
}
