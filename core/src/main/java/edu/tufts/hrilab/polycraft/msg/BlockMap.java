/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class BlockMap extends Msg {
  public String[] blocks;
  public int[] size;
  public int[] origin;

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
    Map<String, Integer> worldTypes = new HashMap<>();
    Set<Predicate> mapAssertions = new HashSet<>();

    //map blocks
    int[] d = {0, 0, 0};
    for (String block : blocks) {
      //Parsing
      String blockName = SymbolResolver.toGridItem(block); //name of block (e.g. oak_log)

      int[] coordinates = {0, 0, 0};
      for (int i = 0; i < 3; i++) {
        coordinates[i] = origin[i] + d[i];
      }
      String[] pos = SymbolResolver.toGridPos(coordinates); //Parses the format x,z

      //types
      int count = worldTypes.getOrDefault(blockName, 0);
      worldTypes.put(blockName, count + 1);

      // at(obj, x, y) predicate
      // don't want to keep track of all air locations
      Predicate posPred = Factory.createPredicate("at", blockName, pos[0], pos[1]);
      Predicate airTypeQuery = Factory.createPredicate("typeobject", blockName, "air");
      if (blockName.equals("air") || Utilities.queryBeliefSupport(airTypeQuery)) {
        if (includeAir) {
          assertions.add(posPred);
        }
      } else {
        assertions.add(posPred);
      }
      mapAssertions.add(posPred);

      if (d[2] < size[2] - 1) {
        d[2] += 1;
      } else {
        d[0] += 1;
        d[2] = 0;
      }
    }

    assertions.addAll(Utilities.generateEntrywayAssertions(mapAssertions));
    assertions.addAll(Utilities.generateAdditionalObjectAssertions(worldTypes));
    return assertions;
  }
}
