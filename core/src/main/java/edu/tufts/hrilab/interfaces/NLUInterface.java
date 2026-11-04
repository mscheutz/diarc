/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.slug.common.Utterance;

/**
 * Base set of methods for natural language understanding in DIARC.
 */
public interface NLUInterface {
  /**
   * Accumulates text, sentence-at-a-time and attempts to parse the incoming Utterance.
   *
   * @return Utterance populated with semantics.
   */
  @TRADEService
  @Action
  Utterance parseUtterance(Utterance incoming);
}
