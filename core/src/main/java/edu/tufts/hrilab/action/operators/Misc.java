/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.operators;

import edu.tufts.hrilab.action.util.Utilities;
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
        log.error("FATAL DEPRECATED. USE ERROR INSTEAD. " + o.toString());
        break;
      case "error":
        log.error(o.toString());
        break;
      case "warn":
        log.warn(o.toString());
        break;
      case "info":
        log.info(o.toString());
        break;
      case "debug":
        log.debug(o.toString());
        break;
      case "trace":
        log.trace(o.toString());
        break;
      default:
        throw new IllegalArgumentException("Invalid logging level: " + level);
    }
  }

  public static boolean isVariable(String str) {
    return Utilities.isScriptVariable(str);
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
