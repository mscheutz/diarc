/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.util.Utilities;
import edu.tufts.hrilab.action.observers.OperatorCheck;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class ObserverDatabase {
  private final static Logger log = LoggerFactory.getLogger(ObserverDatabase.class);

  /**
   * A map of all actions keyed by their observations.
   */
  private final Map<String, List<Pair<Predicate, ActionDBEntry>>> observationsDB = new HashMap<>();

  private final Set<ActionDBEntry> primitives = new HashSet<>();
  private final Set<ActionDBEntry> scripts = new HashSet<>();

  /**
   * Protected so that only the Database can instantiate.
   */
  protected ObserverDatabase() {
    // instantiate core (primitive) observers which are added to this DB via Database.addPrimitiveActions
    new OperatorCheck();
  }

  /**
   * Add an action's observations to the database. This should only be called
   * internally when an ActionDBEntry is added.
   *
   * @param entry the action for which the observations should be added
   */
  protected synchronized void addObservationsToDB(ActionDBEntry entry) {
    if (!entry.getObservations().isEmpty()) {
      List<Class<?>> argTypes = entry.getRequiredInputRolesTypes();
      List<Class<?>> returnTypes = entry.getReturnRolesTypes();
      if (argTypes.size() == 1 && returnTypes.size() == 1
              && Term.class.isAssignableFrom(argTypes.get(0))
              && List.class.isAssignableFrom(returnTypes.get(0))) {

        // hash observer based on advertised observable predicate
        for (Predicate obs : entry.getObservations()) {
          String name = obs.getName();
          List<Pair<Predicate, ActionDBEntry>> map;
          Pair<Predicate, ActionDBEntry> pair = Pair.of(obs, entry);

          log.debug("putting observation into DB: " + obs);

          map = observationsDB.get(name);

          if (map == null) {
            // this observation hasn't been seen before, add it so this action can be looked up
            map = new ArrayList<>(Collections.singletonList(pair));
            observationsDB.put(name, map);
          } else {
            // this observation has been seen before, add DBEntry to the list if it is not already present
            if (!map.contains(pair)) {
              map.add(0, pair);
            }
          }
        }

        // add to primitive/script set
        if (entry.isPrimitive()) {
          primitives.add(entry);
        } else {
          scripts.add(entry);
        }
      } else {
        log.warn("The action '" + entry.getName() + "' cannot be an observer as it is not of form '"
                + entry.getName() + "(Term p) returns List<Map<Variable,Symbol>>'. The action will not be available as "
                + "an observer.");
      }
    }
  }

  /**
   * Remove an action's observations from the database. Should only be used
   * internally.
   *
   * @param entry the action for which the observations should be removed
   */
  protected synchronized void removeObservationsFromDB(ActionDBEntry entry) {
    for (Predicate obs : entry.getObservations()) {
      String name = obs.getName();
      List<Pair<Predicate, ActionDBEntry>> map;
      log.debug("removing observation from DB: " + obs);

      map = new ArrayList<>(observationsDB.get(name));

      for (Pair<Predicate, ActionDBEntry> pa : map) {
        if (pa.getLeft().instanceOf(obs) && pa.getRight().equals(entry)) {
          observationsDB.get(name).remove(pa);
        }
      }

      // remove primitive/script set
      if (entry.isPrimitive()) {
        primitives.remove(entry);
      } else {
        scripts.remove(entry);
      }
    }
  }

  /**
   * Lookup observer by observation predicate.
   *
   * @param obs observation
   * @return list of observers for this observation
   */
  @TRADEService
  public final List<ActionDBEntry> getObservers(Predicate obs) {
    return getObservers(null, obs);
  }

  @TRADEService
  @Action
  public final synchronized List<ActionDBEntry> getObservers(Symbol actor, Predicate obs) {
    log.debug("looking up Observation");

    // if obs is negated, strip off "not" to look up observation
    if (obs.getName().equalsIgnoreCase("not")) {
      Symbol p = obs.getArgs().get(0);
      if (!p.isPredicate()) {
        log.error("[lookupObs] Incorrect predicate form. Inner argument is not a predicate: " + obs);
        return new ArrayList<>();
      }
      obs = (Predicate) p;
    }

    List<Pair<Predicate, ActionDBEntry>> map;
    if (observationsDB.containsKey(obs.getName())) {
      map = observationsDB.get(obs.getName());
    } else {
      map = new ArrayList<>();
    }
    return Utilities.filterEntries(obs, map, actor);
  }

  public final synchronized Set<ActionDBEntry> getPrimitives() {
    Set<ActionDBEntry> allActions = new HashSet<>();
    allActions.addAll(primitives);
    return allActions;
  }

  public final synchronized Set<ActionDBEntry> getScripts() {
    Set<ActionDBEntry> allActions = new HashSet<>();
    allActions.addAll(scripts);
    return allActions;
  }

}
