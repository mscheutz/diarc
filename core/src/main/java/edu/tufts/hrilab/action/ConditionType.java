/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author
 */
package edu.tufts.hrilab.action;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public enum ConditionType {
  PRE,
  OVERALL,
  OBLIGATION;

  private static Logger log = LoggerFactory.getLogger(EffectType.class);

  public static ConditionType fromString(String typeStr) {
    if (typeStr != null) {
      for (ConditionType t : ConditionType.values()) {
        if (typeStr.equalsIgnoreCase(t.name())) {
          return t;
        }
      }
    }
    log.error("Unknown effect type: " + typeStr);
    return null;
  }
}
