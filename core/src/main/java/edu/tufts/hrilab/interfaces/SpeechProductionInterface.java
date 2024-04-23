/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

/**
 Defines a standard set of methods that must be defined in an
 DIARCComponent to send text for speech production.

 Naturally, it is not required to conform or even to use the
 interfaces. However, by explicitly declaring them, it acts
 as a guarantee that robot control code will work mostly as
 expected no matter what DIARCComponent it is connected with.
 */
public interface SpeechProductionInterface {
  /** Speaks appropriate text, blocking.
   * @param text the text to be spoken
   */
  @TRADEService
  @Action
  boolean sayText(String text);

  /** Speaks appropriate text
   * @param text the text to be spoken
   * @param wait whether or not to block until speaking call returns
   */
  @TRADEService
  boolean sayText(String text, boolean wait);

  /** Checks if speech is being produced.
   * @return {@code true} if speech is being produced, {@code false}
   * otherwise
   */
  @TRADEService
  boolean isSpeaking();

  /** Stops an ongoing utterance.
   * @return true if speech is interrupted, false
   * otherwise.
   */
  @TRADEService
  boolean stopUtterance();
}						
