/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.operators;

import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class Misc {

  private final static Logger log = LoggerFactory.getLogger("edu.tufts.hrilab.action.db.Script");

  public static void sleep(long ms) throws OperatorException {
    try {
      Thread.sleep(ms);
    } catch (InterruptedException e) {
      throw new OperatorException("Sleep failed. ", e);
    }
  }

  public static int randomInt(int lower, int upper) {
    double rand = Math.random();
    return (int) Math.floor((upper - lower) * rand) + lower;
  }

  public static double randomDouble(int lower, int upper) {
    double rand = Math.random();
    return ((upper - lower) * rand) + lower;
  }

  public static void log(String level, Object o) {
    switch (level) {
      case "fatal":
        if (o == null) {
          log.error("FATAL DEPRECATED. USE ERROR INSTEAD. null");
        } else {
          log.error("FATAL DEPRECATED. USE ERROR INSTEAD. " + o.toString());
        }
        break;
      case "error":
        if (o == null) {
          log.error("null");
        } else {
          log.error(o.toString());
        }
        break;
      case "warn":
        if (o == null) {
          log.warn("null");
        } else {
          log.warn(o.toString());
        }
        break;
      case "info":
        if (o == null) {
          log.info("null");
        } else {
          log.info(o.toString());
        }
        break;
      case "debug":
        if (o == null) {
          log.debug("null");
        } else {
          log.debug(o.toString());
        }
        break;
      case "trace":
        if (o == null) {
          log.trace("null");
        } else {
          log.trace(o.toString());
        }
        break;
      default:
        throw new IllegalArgumentException("Invalid logging level: " + level);
    }
  }

  public static boolean isVariable(Symbol symbol) {
    return symbol.isVariable();
  }

  /**
   * Return true if a is null, false otherwise.
   * @param a
   * @return
   */
  public static boolean isNull(Object a) {
    return (a == null);
  }

  /**
   * Assign null value to an object.
   * @return
   */
  public static Object setNull() {
    return null;
  }
}
