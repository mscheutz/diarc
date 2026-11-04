/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Effect types.
 * - ALWAYS effects are presumed to hold by virtue of having attempted this action.
 * - SUCCESS effects are presumed to hold only if this action succeeds.
 * - FAILURE effects are presumed to hold only if this action fails.
 * - NONPERFORMANCE effects are presumed to hold only if this action is not performed.
 */
public enum EffectType {
  ALWAYS,
  SUCCESS,
  FAILURE,
  NONPERF;

  private static Logger log = LoggerFactory.getLogger(EffectType.class);

  public static EffectType fromString(String typeStr) {
    if (typeStr != null) {
      for (EffectType t : EffectType.values()) {
        if (typeStr.equalsIgnoreCase(t.name())) {
          return t;
        }
      }
    }
    log.error("Unknown effect type: " + typeStr);
    return null;
  }
}
