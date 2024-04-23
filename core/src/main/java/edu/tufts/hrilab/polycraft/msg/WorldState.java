/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Predicate;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class WorldState extends Msg {

  public BlockInFront blockInFront;
  public Map<String, InventoryItem> inventory;
  public Player player;
  public int[] destinationPos;
  public Map<String, Entities.Entity> entities;
  public BlockMap map;

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(blockInFront.generateAssertions());
    assertions.addAll(new Inventory(inventory).generateAssertions());
    assertions.addAll(player.generateAssertions());
    assertions.addAll(new Entities(entities).generateAssertions());
    assertions.addAll(map.generateAssertions());
    return assertions;
  }

  public Set<Predicate> generateLocationAssertions(boolean includeAir) {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(player.generateAssertions());
    assertions.addAll(new Entities(entities).generateAssertions());
    assertions.addAll(map.generateMapAssertions(includeAir));
    return assertions;
  }
}
