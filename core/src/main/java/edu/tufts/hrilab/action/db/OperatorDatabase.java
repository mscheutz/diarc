/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.db.util.Utilities;
import edu.tufts.hrilab.action.operators.OperatorSymbol;
import edu.tufts.hrilab.action.operators.Operators;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class OperatorDatabase {

  private final static Logger log = LoggerFactory.getLogger(OperatorDatabase.class);

  /**
   * A map of all the operators keyed by their type.
   */
  private final Map<String, List<OperatorDBEntry>> operatorDB = new HashMap<>();

  /**
   * Protected so that only the Database can instantiate.
   */
  protected OperatorDatabase() {
    // Add core operators available in action to the database
    addOperators(Operators.getClasses());
  }

  protected void addOperators(Class[] operatorClasses) {
    // Add available operators (public methods)
    for (Class c : operatorClasses) {
      for (Method m : c.getDeclaredMethods()) {
        addOperator(m);
      }
    }
  }

  protected void addOperator(Method m) {
    if (Modifier.isPublic(m.getModifiers()) && Modifier.isStatic(m.getModifiers())) {
      String name;
      if (m.isAnnotationPresent(OperatorSymbol.class)) {
        name = m.getDeclaredAnnotation(OperatorSymbol.class).value();
      } else {
        name = m.getName();
      }

      if (!name.isEmpty()) {
        putOperator(new OperatorDBEntry(name, m));
      } else {
        log.error("Cannot add operator with empty symbol.");
      }
    }
  }

  /**
   * Insert the operator into the database. The operator is keyed by its type.
   *
   * @param entry the database entry defining this action
   */
  private final void putOperator(OperatorDBEntry entry) {
    log.trace("Adding new operator to DB: " + entry);
    Utilities.putEntry(operatorDB, entry);
  }

  /**
   * Lookup operator by type. Returns last added operator if more than one
   * operator with type.
   *
   * @param type the type of the operator to look up
   * @return the requested operator (last added), if found, null otherwise
   */
  public final OperatorDBEntry getOperator(String type) {
    OperatorDBEntry odb = Utilities.getEntry(operatorDB, type);
    if (odb == null) {
      log.warn("Could not find operator for type: " + type);
    }
    return odb;
  }

  /**
   * Lookup operator by type and roles.
   *
   * @param type the type of the operator to look up
   * @return the requested operator, if found, null otherwise
   */
  @TRADEService
  public final OperatorDBEntry getOperator(String type, List<Class<?>> inputRoleTypes) {
    OperatorDBEntry odb = Utilities.getEntry(operatorDB, type, inputRoleTypes);
    if (odb == null) {
      log.warn("Could not find operator for type " + type + " with role types " + inputRoleTypes);
    }
    return odb;
  }

  /**
   * Lookup operator by type and roles. Do not print error if lookup fails.
   *
   * @param type the type of the operator to look up
   * @return the requested operator, if found, null otherwise
   */
  public final OperatorDBEntry getOperatorSilent(String type, List<Class<?>> roles) {
    return Utilities.getEntry(operatorDB, type, roles);
  }
}
