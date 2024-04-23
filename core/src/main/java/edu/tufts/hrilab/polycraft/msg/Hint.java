/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class Hint extends Msg {

  public List<Map<String,String>> worldElements;
  public List<String> commands;
  public String impact;

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(generateHintAssertions());
    return assertions;
  }

  private Set<Predicate> generateHintAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    String name = "";
    String argument = "";
    for (int j = 0; j < worldElements.size(); j++) {
      Map<String,String> hint = worldElements.get(j);

      String[] nameSplit = hint.get("name").split("\\."); // e.g., "item.item.hatchetWood", "entity.polycraft.Pogoist.name"
      if (nameSplit.length<3) {
        name = SymbolResolver.toGridItem(hint.get("name"));
      } else {
        log.warn("Hint refers to actor");
        name = SymbolResolver.toGridItem(nameSplit[2].toLowerCase());
      }

      if (hint.keySet().contains("variant")){
        argument = hint.get("variant")+"_"+name;
      } else {
        argument = name;
      }
    }

    for (String command : commands) {
      String action;
      if (!argument.isEmpty()) {
        action = SymbolResolver.toAction(command, argument);
      } else {
        action = SymbolResolver.toAction(command);
      }

      if (action!=null) {
        ActionDBEntry abd = Database.getActionDB().getAction(action);
        if (abd != null) {
          try {
            String novelAction = TRADE.getAvailableService(new TRADEServiceConstraints().name("fillAction")).call(String.class,abd, argument, false);
            Predicate explore = Factory.createPredicate(novelAction);
            log.debug("Asserting " + explore.toString());
            Predicate predicate = Factory.createPredicate("toExplore",argument,novelAction);
            assertions.add(predicate);
          } catch (TRADEException e) {
            log.error("Cannot fill action",e);
          }
        }
      }
    }

    return assertions;
  }

}
