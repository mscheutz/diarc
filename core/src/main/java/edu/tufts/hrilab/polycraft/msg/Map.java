/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Gson can't directly populate this class because it contains a java.util.Map, so it must
 * be populated manually after the JSON has been parsed.
 */
public class Map extends Msg {
  public java.util.Map<String, MapBlock> map;

  /**
   * Map constructor.
   *
   * @param map map data that was parsed from JSON format
   */
  public Map(java.util.Map<String, MapBlock> map) {
    this.map = map;
  }

  @Override
  public Set<Predicate> generateAssertions() {
    return generateMapAssertions(false);
  }

  /**
   * Generate all predicates from the map data.
   *
   * @return
   */
  public Set<Predicate> generateMapAssertions(boolean includeAir) {

    // for Predicate assertions
    Set<Predicate> assertions = new HashSet<>();
    java.util.Map<String, Integer> worldTypes = new HashMap<>();

    Set<Predicate> mapAssertions = new HashSet<>();

    //map blocks
    for (java.util.Map.Entry<String, MapBlock> entry : map.entrySet()) {
      MapBlock block = entry.getValue();

      //Parsing
      String blockName = SymbolResolver.toGridItem(block.name, block.variant); // name of block: variant_name (e.g. oak_log)

      // variant -- create subtype
      if (block.variant != null) {
        Predicate subtype = Factory.createPredicate("subtype", new Symbol(blockName), new Symbol(SymbolResolver.toGridItem(block.name)));
        assertions.add(subtype);
      }

      // properties
      if (block.open != null && block.open.equals("true")) {
        blockName = "open_" + blockName;

        Predicate subtype = Factory.createPredicate("subtype", new Symbol(blockName), new Symbol(SymbolResolver.toGridItem(block.name)));
        assertions.add(subtype);
      }

      // at(obj, x, y) predicate
      // don't want to keep track of all air locations
      String[] pos = SymbolResolver.toGridPos(entry.getKey().split(",")); //Parses the format x,z
      Predicate posPred = Factory.createPredicate("at", blockName, pos[0], pos[1]);
      Predicate airTypeQuery = Factory.createPredicate("typeobject", blockName, "air");
      if (blockName.equals("air") || Utilities.queryBeliefSupport(airTypeQuery)) {
        if (includeAir) {
          assertions.add(posPred);
        }
      } else {
        assertions.add(posPred);

        // types -- don't care to count air
        int count = worldTypes.getOrDefault(blockName, 0);
        worldTypes.put(blockName, count + 1);
      }
      mapAssertions.add(posPred);
    }

    assertions.addAll(Utilities.generateEntrywayAssertions(mapAssertions));
    assertions.addAll(Utilities.generateAdditionalObjectAssertions(worldTypes));
    return assertions;
  }

}
