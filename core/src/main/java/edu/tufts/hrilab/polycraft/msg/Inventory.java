/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Gson can't directly populate this class because it contains a java.util.Map, so it must
 * be populated manually after the JSON has been parsed.
 */
public class Inventory extends Msg {
  public Map<String, InventoryItem> inventory;

  /**
   * Inventory constructor.
   *
   * @param inventory data that was parsed from JSON format
   */
  public Inventory(Map<String, InventoryItem> inventory) {
    this.inventory = inventory;
  }

  @Override
  public Set<Predicate> generateAssertions() {
    return generateInventoryAssertions();
  }

  /**
   * Generate all predicates from the inventory data.
   *
   * @return
   */
  public Set<Predicate> generateInventoryAssertions() {
    // for Predicate assertions
    Set<Predicate> assertions = new HashSet<>();

    // Holding
    InventoryItem holding = inventory.remove("selectedItem");
    String holdingItem;
    if (holding == null || holding.item == null || holding.item.equals("")) {
      holdingItem = "air";
    } else {
      holdingItem = SymbolResolver.toGridItem(holding.item, holding.variant);
      if (holding.variant != null) {
        Predicate typePred = Factory.createPredicate("subtype", holdingItem, SymbolResolver.toGridItem(holding.item));
        assertions.add(typePred);
    }
    }
    log.debug("holding: " + holdingItem);
    assertions.add(Factory.createPredicate("holding", actor, Factory.createSymbol(holdingItem)));

    //Inventory
    for (Map.Entry<String, InventoryItem> entry : inventory.entrySet()) {
      InventoryItem item = entry.getValue();
      String itemName = SymbolResolver.toGridItem(item.item, item.variant);

      Predicate invPred = Factory.createPredicate("fluent_equals", "inventory(self," + itemName + ")", Double.toString(item.count));
      assertions.add(invPred);

      Predicate objPred = Factory.createPredicate("constant", itemName, itemName);
      assertions.add(objPred);

      Predicate typePred = Factory.createPredicate("subtype", itemName, "physobj");
      assertions.add(typePred);

      if (item.variant != null) {
        Predicate subtypePred = Factory.createPredicate("subtype", itemName, SymbolResolver.toGridItem(item.item));
        assertions.add(subtypePred);
      }
    }

    return assertions;
  }
}
