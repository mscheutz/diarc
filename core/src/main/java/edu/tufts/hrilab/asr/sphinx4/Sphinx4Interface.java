/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.asr.sphinx4;

import ai.thinkingrobots.trade.TRADEService;

/**
 * Defines a standard set of methods that must be defined in an DIARCComponent
 * that utilizes Sphinx4 speech recognition.
 */
public interface Sphinx4Interface {

  @TRADEService
  void setGrammar(String path);

  @TRADEService
  void setConfig(String path);


  //GUI related methods

  /**
   * Set control state to one of {CONFIRM, ACCEPT, REJECT}. GUI only method.
   *
   * @param state one of {"CONFIRM", "ACCEPT", "REJECT"}
   */
  @TRADEService
  void setControlState(Sphinx4Component.ControlState state);

  /**
   * get current control state, one of {CONFIRM, ACCEPT, REJECT}. GUI only method.
   */
  @TRADEService
  Sphinx4Component.ControlState getControlState();

  /**
   * Accept last recognition result and pass to parser. This is only for
   * the CONFIRM control state.
   */
  @TRADEService
  void acceptUtterance();

  /**
   * Reject last recognition result. This is only for the CONFIRM control state.
   */
  @TRADEService
  void rejectUtterance();

  /**
   * Get the next recognition result from the recognition queue while in CONFIRM
   * control state.
   *
   */
  @TRADEService
  String getOnDeckText();

  /**
   * Clear recognition queue. Only for use while in CONFIRM control state.
   */
  @TRADEService
  void clearRecogntionHistory();
}
