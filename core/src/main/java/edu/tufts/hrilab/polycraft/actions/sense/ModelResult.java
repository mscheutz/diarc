/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions.sense;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import com.google.gson.Gson;
import edu.tufts.hrilab.polycraft.msg.InventoryItem;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class ModelResult extends SenseAll {

  protected Symbol actor = Factory.createSymbol("self");

  public ModelResult(String rawResponse) {
    super();

    this.rawResponse = rawResponse;
    log.debug("NNMODEL - RAW " + rawResponse);
    this.receivedResponse = true;
    this.parsedResponse = new Gson().fromJson(this.rawResponse, SenseAll.ParsedResponse.class);
  }

  public Set<Predicate> getAssertions() {
    Set<Predicate> assertions = new HashSet<>();

    try {
      Set<Predicate> player_assertions = parsedResponse.player.generateAssertions();
      assertions.addAll(player_assertions);
      log.debug("NNModel - Made player predicates from nn result");
      for (Predicate p : player_assertions) {
        log.debug("NNMODEL predicate - " + p.toString());
      }
    } catch (Exception e) {
      log.debug("NNModel - cannot make player predicates from result", e);
    }

    try {
      Set<Predicate> inventory_assertions = generatePartialInventoryAssertions();
      assertions.addAll(inventory_assertions);
      log.debug("NNModel - Made inventory predicates from result");
      for (Predicate p : inventory_assertions) {
        log.debug("NNMODEL predicate - " + p.toString());
      }
    } catch (Exception e) {
      log.error("NNModel - cannot make inventory predicates from result", e);
    }

    return assertions;
  }

  /**
   * Generate predicates from partial inventory data.
   *
   * @return
   */
  protected Set<Predicate> generatePartialInventoryAssertions() {

    // for Predicate assertions
    Set<Predicate> assertions = new HashSet<>();
    Map<String, Integer> invTypes = new HashMap<>();

    //Holding
    if (parsedResponse.inventory.containsKey("selectedItem")) {

      InventoryItem holding = parsedResponse.inventory.remove("selectedItem");
      String holdingItem;
      if (holding == null || holding.item == null || holding.item.equals("")) {
        holdingItem = "air";
      } else {
        holdingItem = SymbolResolver.toGridItem(holding.item);
      }
      assertions.add(Factory.createPredicate("holding", actor, Factory.createSymbol(holdingItem)));
    }


    //Inventory
    for (Map.Entry<String, InventoryItem> entry : parsedResponse.inventory.entrySet()) {
      InventoryItem item = entry.getValue();
      if (item.item != null) {
        String itemName = SymbolResolver.toGridItem(item.item);
        invTypes.put(itemName, item.count);
      }
    }

    //assert inventory fluents
    for (Map.Entry<String, Integer> entry : invTypes.entrySet()) {
      String itemName = entry.getKey();
      log.info("NNMode - itemName " + itemName);
      Predicate invItem = Factory.createPredicate("inventory", "self", itemName);
      Symbol invQuant = Factory.createSymbol(Double.toString(entry.getValue()));
      Predicate invPred = Factory.createPredicate("fluent_equals", invItem, invQuant);
      assertions.add(invPred);

      String key = entry.getKey();
      Predicate objPred = Factory.createPredicate("constant", key, key);
      assertions.add(objPred);

      Predicate typePred = Factory.createPredicate("subtype", key, "physobj");
      assertions.add(typePred);
    }

    return assertions;
  }
}
