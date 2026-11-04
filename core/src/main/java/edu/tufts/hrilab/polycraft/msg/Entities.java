/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;
import org.apache.commons.lang3.StringUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Gson can't directly populate this class because it contains a java.util.Map, so it must
 * be populated manually after the JSON has been parsed.
 */
public class Entities extends Msg {
  public Map<String, Entity> entities;

  /**
   * Entities constructor.
   *
   * @param entities entities data that was parsed from JSON format
   */
  public Entities(Map<String, Entity> entities) {
    this.entities = entities;
  }

  public class Entity {
    public int[] pos; // [x,y,z]
    public String type; //"EntityItem"
    public String name; //
    public int id; // "426"
    public String item; // minecraft:sapling
  }

  @Override
  public Set<Predicate> generateAssertions() {
    return generateEntityAssertions();
  }

  public Set<Predicate> generateEntityAssertions() {
    // for Predicate assertions
    Set<Predicate> assertions = new HashSet<>();
    Map<String, Integer> worldCounts = new HashMap<>();
    Map<String, Integer> agentTypeCounts = new HashMap<>();

    //Entities
    for (Map.Entry<String, Entity> entry : entities.entrySet()) {
      Entity entity = entry.getValue();

      // get entity name
      String entityName = Utilities.getEntityName(entity);

      // get rid of all spaces
      entityName = entityName.replaceAll("\\s", "");

      // filter out stuff that shouldn't be in the json
      if (entityName.equals("ExperienceOrb")) {
        continue;
      }

      // if first letter is upper case, make it lower case -- don't want to break prolog
      if (Character.isUpperCase(entityName.codePointAt(0))) {
        entityName = StringUtils.lowerCase(entityName);
      }

      // create at(entity, x, y)
      String[] pos = SymbolResolver.toGridPos(entity.pos); //Parses the format x,y,z
      Predicate posPred = Factory.createPredicate("at", entityName, pos[0], pos[1]);
      log.debug("Adding entity at " + posPred);
      assertions.add(posPred);

      // create subtype(entity, x)
      String subtype;
      if (entity.item == null) {
        // agent entity
        subtype = Utilities.getActorType(entity);

        // assume this is an actor -- subtype(subtype, actor)
        Predicate objectPred = Factory.createPredicate("subtype", subtype, "agent");
        assertions.add(objectPred);

        // constant(entity, subtype)
        objectPred = Factory.createPredicate("constant", entityName, subtype);
        assertions.add(objectPred);

        // count agents of each type
        int count = agentTypeCounts.getOrDefault(subtype, 0);
        agentTypeCounts.put(subtype, count + 1);
      } else {
        // floating entity
        subtype = "physobj";
        Predicate floatPred = Factory.createPredicate("floating", entityName); // TODO: is this always the case?
        assertions.add(floatPred);

        // create constant(entity, entity)
        Predicate objectPred = Factory.createPredicate("constant", entityName, entityName);
        assertions.add(objectPred);

        // create subtype(entity, physobj)
        Predicate typePred = Factory.createPredicate("subtype", entityName, subtype);
        assertions.add(typePred);

        // count entity
        int count = worldCounts.getOrDefault(entityName, 0);
        worldCounts.put(entityName, count + 1);
      }
    }

    // add fluent_equals(numberOfType(subtype),count) for actors
    for (Map.Entry<String, Integer> entry : agentTypeCounts.entrySet()) {
      String itemName = entry.getKey();
      Predicate numTypePred = Factory.createPredicate("fluent_equals", "numberOfType(" + itemName + ")", Double.toString(entry.getValue()));
      assertions.add(numTypePred);
    }

    // add fluent_equals(world(entityName),count) for entities
    for (Map.Entry<String, Integer> entry : worldCounts.entrySet()) {
      String itemName = entry.getKey();
      Predicate worldPred = Factory.createPredicate("fluent_equals", "world(" + itemName + ")", Double.toString(entry.getValue()));
      assertions.add(worldPred);
    }

    return assertions;
  }

  public List<Map<Variable, Symbol>> getAtBindings(Variable objVar, Variable xVar, Variable yVar) {

    List<java.util.Map<Variable, Symbol>> atBindings = new ArrayList<>();
    for (Map.Entry<String, Entities.Entity> entry : entities.entrySet()) {
      Entities.Entity entity = entry.getValue();
      // get entity name
      String entityName = Utilities.getEntityName(entity);

      // get entity position
      String[] pos = SymbolResolver.toGridPos(entity.pos); //Parses the format x,y,z

      // pathfinding map (not assertions, but here for efficiency)
      log.debug(entityName + " at " + pos[0] + "," + pos[1]);
      Map<Variable, Symbol> atBinding = new HashMap<>();
      atBinding.put(objVar, Factory.createSymbol(entityName));
      atBinding.put(xVar, Factory.createSymbol(pos[0]));
      atBinding.put(yVar, Factory.createSymbol(pos[1]));
      atBindings.add(atBinding);
    }

    return atBindings;
  }
}
