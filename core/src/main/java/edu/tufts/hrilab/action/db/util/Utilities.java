/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.util;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.DBEntry;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Collectors;

import static edu.tufts.hrilab.action.util.Utilities.isAssignable;
import static edu.tufts.hrilab.action.util.Utilities.isBooleanType;
import static edu.tufts.hrilab.action.util.Utilities.isNumericType;

public class Utilities {
  private final static Logger log = LoggerFactory.getLogger(Utilities.class);

  /**
   * Lookup by type.
   *
   * @param db   the database in which to look up
   * @param type the type of the entity to look up
   * @return list of entries with matching type
   */
  public static  <T extends DBEntry> List<T> getEntries(Map<String, List<T>> db, String type) {
    List<T> entries;
    // Get db entries with type
    entries = db.get(type);

    return entries;
  }

  /**
   * Lookup by type and roles.
   *
   * @param db             the database in which to look up
   * @param type           the type of the entity to look up
   * @param inputRoleTypes the java types of the input roles of the Entry to look up
   * @return list of entries with matching type and roles, best match first.
   */
  public static <T extends DBEntry> List<T> getEntries(Map<String, List<T>> db, String type,
                                                          List<Class<?>> inputRoleTypes) {
    List<T> entries = getEntries(db, type);
    List<T> matchingEntries = new ArrayList<>();

    if (entries != null && !entries.isEmpty()) {
      return findMatchingEntries(entries, inputRoleTypes);
    }
    return matchingEntries;
  }

  /**
   * Lookup by type. Returns last added entry if more than one entry with type.
   *
   * @param db   the database in which to look up
   * @param type the type of the entity to look up
   * @return the (last) requested entry, if found, null otherwise
   */
  public  static <T extends DBEntry> T getEntry(Map<String, List<T>> db, String type) {
    List<T> entries = getEntries(db, type);
    if (entries != null && !entries.isEmpty()) {
      return entries.get(0);
    }
    return null;
  }

  /**
   * Lookup by type and roles. Tries to find best suited entry according to
   * provided roles.
   *
   * @param db             the database in which to look up
   * @param type           the type of the entity to look up
   * @param inputRoleTypes the java types of the input roles of the Entry to look up
   * @return the requested entry, if found, null otherwise
   */
  public static <T extends DBEntry> T getEntry(Map<String, List<T>> db, String type, List<Class<?>> inputRoleTypes) {
    List<T> entries = getEntries(db, type, inputRoleTypes);

    if (!entries.isEmpty()) {
      return entries.get(0); // Return best matching entry
    }

    return null;
  }

  /**
   * Lookup by type, roles and agent. Tries to find best suited entry according
   * to provided roles.
   *
   * @param db             the database in which to look up
   * @param type           the type of the entity to look up
   * @param actor          the actor affected by the entity to look up
   * @param inputRoleTypes the java types of the input roles of the action to look up
   * @return the requested entry, if found, null otherwise
   */
  public static ActionDBEntry getEntry(Map<String, List<ActionDBEntry>> db,
                                          String type, Symbol actor, List<Class<?>> inputRoleTypes) {
    List<ActionDBEntry> entries = getEntries(db, type, inputRoleTypes);

    if (entries.size() > 0) {
      // Sort by affected agents
      List<ActionDBEntry> entriesByAgent = new ArrayList<>();
      List<ActionDBEntry> entriesToRemove = new ArrayList<>();

      for (ActionDBEntry adb : entries) {
        if (adb.getAgents().contains(actor)) {
          entriesByAgent.add(adb);
        } else if (!adb.getAgents().isEmpty()) {
          entriesToRemove.add(adb); // Remove entries specific to other agents
        }
      }

      // Replace entries list if there are agent specific actions
      if (!entriesByAgent.isEmpty()) {
        entries = entriesByAgent;
      } else {
        entries.removeAll(entriesToRemove);
      }

      // Return best match
      if (!entries.isEmpty()) {
        return entries.get(0);
      }
    }
    return null;
  }

  /**
   * Put a new entry in the database.
   *
   * @param db    the database in which to put the entry
   * @param entry the new entry
   */
  public static <T extends DBEntry> void putEntry(Map<String, List<T>> db, T entry) {
    // add entry to DB

    if (!db.containsKey(entry.getType())) {
      db.put(entry.getType(), new ArrayList<>());
    }
    db.get(entry.getType()).add(0, entry);
  }

  /**
   * Finds the matching DBEntries for the provided role types.
   *
   * @param <T>            type of DBEntry
   * @param entries        DBEntries.
   * @param inputRoleTypes the java types of the input roles of the Entry to look up
   * @return list of matching DBEntries.
   */
  @TRADEService
  public final static <T extends DBEntry> List<T> findMatchingEntries(List<T> entries, List<Class<?>> inputRoleTypes) {
    Map<Integer, List<T>> matches = new TreeMap<>(Collections.reverseOrder());

    for (T entry : entries) {
      List<Class<?>> required = entry.getRequiredInputRolesTypes();
      List<Class<?>> optional = entry.getOptionalInputRolesTypes();
      List<Class<?>> tmpInputArgTypes;

      // attempt to deal with varargs entries by dynamically generating a new DBEntry
      // with fully expanded varargs to match provided args (e.g., Object[] --> Object, Object)
//      if (entry.isVarArgs()) {
//        T varArgEntry = (T) entry.getDynamicVarArgsEntry(inputRoleTypes);
//
//        if (varArgEntry == null) {
//          // could not be converted to DBEntry with varArgs
//          continue;
//        }
//        entry = varArgEntry;
//        required = entry.getRequiredInputRolesTypes();
//        optional = entry.getOptionalInputRolesTypes();
//      }
      if (entry.isVarArgs()) {
        // get vararg java type (e.g., String if String[])
        Class<?> varArgType = required.get(required.size() - 1);
        Class<?> varArgComponentType = required.get(required.size() - 1).getComponentType();
        tmpInputArgTypes = new ArrayList<>();
        for (int i = 0; i < required.size() - 1; ++i) {
          tmpInputArgTypes.add(inputRoleTypes.get(i));
        }

        if (!optional.isEmpty()) {
          log.warn("It should not be possible to have optional input args and varargs.");
          continue;
        }

        // check that all vararg input arg types can be assigned to entry's vararg type
        for (int i = required.size() - 1; i < inputRoleTypes.size(); ++i) {
          if (!isAssignable(inputRoleTypes.get(i), varArgComponentType)) {
            continue;
          }
        }
        tmpInputArgTypes.add(varArgType);

      } else {
        // no varargs
        tmpInputArgTypes = inputRoleTypes;
      }

      // Check that enough arguments where provided (>= req), but not too many (<= req+opt).
      if (tmpInputArgTypes.size() >= required.size() && tmpInputArgTypes.size() <= (required.size() + optional.size())) {
        int exact = 0;
        int compatible = 0;

        Iterator<Class<?>> input = tmpInputArgTypes.iterator();
        Iterator<Class<?>> req = required.iterator();
        Iterator<Class<?>> opt = optional.iterator();

        while (req.hasNext() && input.hasNext()) { // Ordering is important here (provided >= required), otherwise OoB.
          Class r = req.next(); // Type of the required argument
          Class p = input.next(); // Type of the provided argument
          exact += (p.equals(r)) ? 1 : 0;
          compatible += (isAssignable(p, r)) ? 1 : 0;
        }

        // If we have a match for all required arguments, look at optional arguments. Note: compatible >= exact.
        if (compatible >= required.size()) {
          while (input.hasNext() && opt.hasNext()) { // Ordering is important here (provided <= required+optional), ...
            Class o = opt.next(); // Type of the optional argument
            Class p = input.next(); // Type of the provided argument
            exact += (p.equals(o)) ? 1 : 0;
            if (isAssignable(p, o)) {
              compatible++;
            } else {
              compatible = -1; // Wrong type. Flag this entry as incompatible.
              break;
            }
          }

          // Check that the entry is compatible
          if (compatible != -1) {
            // Add to map, indexed by number of exact type matches.
            if (matches.get(exact) == null) {
              matches.put(exact, new ArrayList<>());
            }
            matches.get(exact).add(entry);
          }
        }
      }
    }

    // Extract indices by descending exact match count
    List<T> orderedMatches = new ArrayList<>();
    for (Integer i : matches.keySet()) {
      orderedMatches.addAll(matches.get(i));
    }

    if (orderedMatches.size() == 0) {
      log.debug("Could not find matching signature for roles: " + inputRoleTypes.toString());
    }

    return orderedMatches;
  }

  /**
   * Filter possible ActionDBEntries found for predicate
   *
   * @param pred input predicate
   * @param map  list of possible Entries
   * @return list of ActionDBEntries
   */
  public static List<ActionDBEntry> filterEntries(Predicate pred, List<Pair<Predicate, ActionDBEntry>> map, Symbol actor) {
    if (map != null) {
      // Search for exact post condition match
      List<Pair<Predicate, ActionDBEntry>> actions = map.stream()
              .filter(x -> pred.instanceOf(x.getLeft()))
              .collect(Collectors.toList());

      // Filter based on argument type. Basically, it doesn't make sense to allow the use of actions
      // that have non-string, non-symbol, non-numeric, or non-boolean arguments, as those values cannot be passed in via a goal predicate.
      // TODO: consider combining this javaType check with code in edu.tufts.hrilab.action.util.Utilities
      actions = actions.stream()
              .filter(a ->
                      a.getValue().getInputRolesTypes().stream().allMatch(c ->
                              c.isAssignableFrom(String.class) || c.isAssignableFrom(Predicate.class)
                              ||isNumericType(c) || isBooleanType(c)
                      )
              ).collect(Collectors.toList());

      // Filter agent specific actions
      List<ActionDBEntry> byAgent = new ArrayList<>();
      List<ActionDBEntry> byDefault = new ArrayList<>();

      for (Pair<Predicate, ActionDBEntry> entry : actions) {
        List<Symbol> entryActors = entry.getRight().getAgents();
        if (!entryActors.isEmpty()) {
          // Action is agent specific.
          if (actor == null) {
            actor = getAgentFromBindings(entry.getLeft(), pred);
          }
          if (actor != null && entryActors.contains(actor)) {
            byAgent.add(entry.getRight());
          } else if (actor == null || actor.isVariable()) { // used to get all entries for loading performance assessment models
            // FIXME: need better way to load performance models
            byDefault.add(entry.getRight());
          }
        } else {
          byDefault.add(entry.getRight());
        }
      }

      if (!byAgent.isEmpty()) {
        return byAgent; // Return agent specific actions
      }
      return byDefault; // Return general actions

    } else {
      log.warn("Found no entries for " + pred.getName() + " in DB");
    }
    return null;
  }

  /**
   * Helper method getting an agent remoteMethodName from a post condition and an instance
   * of it.
   *
   * @param postcond generic post condition
   * @param instance instance of above post condition
   * @return agent remoteMethodName
   */
  private static Symbol getAgentFromBindings(Predicate postcond, Predicate instance) {
    Map<Variable, Symbol> bindings = postcond.getBindings(instance);

    // look for ?actor binding regardless of ?actor semantic type
    List<Symbol> actors = bindings.entrySet().stream().filter(e -> e.getKey().getName().equals("?actor")).
            map(e -> e.getValue()).collect(Collectors.toList());

    if (actors.isEmpty()) {
      log.warn("No ?actor found in: " + instance);
      return null;
    } else if (actors.size() == 1) {
      return actors.get(0);
    } else {
      log.warn("More than one ?actor found in: " + instance);
      return actors.get(0);
    }
  }

  /**
   * Checks if the java class and semantic type of the input arg matches the java class and semantic type of the argument in teh DB entry signature.
   * @param signatureArg argument in the signature of the db entry
   * @param inputArg potential input argument to the DB entry
   * @return true if the class and semantic type of the input arg match the signature arg
   */
  public static boolean argMatches(ActionBinding signatureArg, Object inputArg){
    boolean matches = isAssignable(inputArg.getClass(),signatureArg.getJavaType());
    if (!matches) return false;
    //TODO:brad what would it mean for something that isn't a Symbol to have a semantic type?
    if(isAssignable(signatureArg.getJavaType(),Symbol.class)){
      if(signatureArg.getSemanticType().equals("")){
        //TODO:brad implement actual type checking here, not just an exact match
        matches= signatureArg.getSemanticType().equals(((Symbol)inputArg).getType());
      }
    }
    return matches;
  }

}
