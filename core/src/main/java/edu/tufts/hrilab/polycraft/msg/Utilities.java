/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.belief.provers.prolog.Prolog;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class Utilities {
  static private Logger log = LoggerFactory.getLogger(Utilities.class);

  /**
   * Generate entryway(x,y) assertions for navigation between rooms.
   * 
   * @param mapAssertions
   * @return
   */
  static public Set<Predicate> generateEntrywayAssertions(Set<Predicate> mapAssertions) {
    // calculate current room boundaries based on wall locations
    Set<Predicate> assertions = new HashSet<>();
    int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE;
    int maxX = 0, maxY = 0;
    for (Predicate wall : mapAssertions) {
      int x = Integer.parseInt(wall.get(1).getName());
      int y = Integer.parseInt(wall.get(2).getName());
      if (x < minX) {
        minX = x;
      }
      if (x > maxX) {
        maxX = x;
      }
      if (y < minY) {
        minY = y;
      }
      if (y > maxY) {
        maxY = y;
      }
    }

    // populate prolog instance with map assertions
    Prolog prolog = new Prolog();
    mapAssertions.forEach(assertion -> prolog.assertBelief(assertion));

    List<Integer> xedges = Arrays.asList(minX, maxX);
    List<Integer> yedges = Arrays.asList(minY, maxY);
    for (int x = minX; x <= maxX; x++) {
      for (int y : yedges) {
        if (isEntryway(prolog, x, y)) {
          assertions.add(new Predicate("entryway", Factory.createSymbol(String.valueOf(x)), Factory.createSymbol(String.valueOf(y))));
          }
        }
      }
    for (int y = minY; y <= maxY; y++) {
      for (int x : xedges) {
        if (isEntryway(prolog, x, y)) {
          assertions.add(new Predicate("entryway", Factory.createSymbol(String.valueOf(x)), Factory.createSymbol(String.valueOf(y))));
    }
          }
        }
    return assertions;
      }
  /**
   * Check if x,y location is an entryway to another room.
   *
   * @param prolog
   * @param x
   * @param y
   * @return
   */
  static private boolean isEntryway(Prolog prolog, int x, int y) {
    Symbol[] entryTypes = {Factory.createSymbol("air"), Factory.createSymbol("door")}; // possible entryway types

    Variable var = Factory.createVariable("OBJ");
    Predicate query = Factory.createPredicate("at", var, Factory.createSymbol(String.valueOf(x)), Factory.createSymbol(String.valueOf(y)));
    List<Map<Variable, Symbol>> bindings = prolog.queryBelief(query);
    if (!bindings.isEmpty() && bindings.get(0).containsKey(var)) {
      Symbol object = bindings.get(0).get(var);
      for (Symbol type : entryTypes) {
        Predicate typeQuery = Factory.createPredicate("typeobject", object, type);
        if (object.equals(type) || queryBeliefSupport(typeQuery)) {
          return true;
    }
      }
  }

    return false;
  }

  /**
   * Local wrapper around querySupport call to Belief to handle try-catch
   * to make usage in conditionals less ugly.
   * @param query
   * @return
   */
  static public boolean queryBeliefSupport(Predicate query) {
    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, query);
    } catch (TRADEException e) {
      log.error("Error querying belief for: " + query, e);
    }

    return false;
  }

  /**
   * Generate additional assertions based on world object counts (constant, subtype, fluent_equals(world(obj), count), etc)
   * @param worldTypes
   * @return
   */
  static public Set<Predicate> generateAdditionalObjectAssertions(java.util.Map<String,Integer> worldTypes){
    //assert types, objects, and world fluents
    Set<Predicate> assertions = new HashSet<>();
    for (java.util.Map.Entry<String, Integer> entry : worldTypes.entrySet()) {
      String itemName = entry.getKey();
      Predicate objPred = Factory.createPredicate("constant", itemName, itemName);
      assertions.add(objPred);

      Predicate typePred = Factory.createPredicate("subtype", itemName, "physobj");
      assertions.add(typePred);

        Predicate worldPred = Factory.createPredicate("fluent_equals", "world(" + itemName + ")", Double.toString(entry.getValue()));
        assertions.add(worldPred);
      }

    return assertions;
  }

  /**
   * Create the entity name, using item field if it exists, and name + id if it doesn't.
   *
   * @param entity
   * @return
   */
  static public String getEntityName(Entities.Entity entity) {
    if (entity.item == null) {
      // agent entity
      String[] nameSplit = entity.name.split("\\."); // e.g., "item.item.hatchetWood", "entity.polycraft.Pogoist.name"
      if (nameSplit.length<3) {
        log.warn("Entity named: "+entity.name);
        return SymbolResolver.toGridItem(entity.name);
      }
      return SymbolResolver.toGridItem(nameSplit[2].toLowerCase()) + "_" + entity.id;
    } else {
      // floating entity
      return SymbolResolver.toGridItem(entity.item);
    }
  }

  static public String getActorType(Entities.Entity entity) {
    if (entity.type != null && entity.type.startsWith("Entity")) {
      return entity.type.substring(6).toLowerCase();
    } else {
      return "unknown";
    }
  }
}
