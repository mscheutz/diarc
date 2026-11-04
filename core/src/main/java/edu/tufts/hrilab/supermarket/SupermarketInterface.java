/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.fol.Symbol;

public interface SupermarketInterface {

  Integer getSimPort();

  @TRADEService
  @Action
  Justification nop();

  @TRADEService
  @Action
  Justification startGame();

  @TRADEService
  @Action
  Justification resetGame();

  @TRADEService
  @Action
  Justification goNorth();

  @TRADEService
  @Action
  Justification goSouth();

  @TRADEService
  @Action
  Justification goEast();

  @TRADEService
  @Action
  Justification goWest();

  @TRADEService
  @Action
  Justification toggleShoppingCart();

  @TRADEService
  @Action
  Justification interactWithObject();

  @TRADEService
  @Action
  Justification goDir(SupermarketObservation.Direction dir);

  @TRADEService
  @Action
  Justification rotateCW();

  @TRADEService
  @Action
  Justification rotateCCW();

  @TRADEService
  @Action
  Justification pickup(Symbol goal);

  //todo: Should this be moved to a different class, such as SupermarketAgent?
  @TRADEService
  @Action
  Justification reactive_nav(Symbol goal);

  @TRADEService
  @Action
  Justification cancelInteraction();

  @TRADEService
  @Action
  SupermarketObservation getLastObservation();

  @TRADEService
  @Action
  Symbol lastCart();

  String getAgentName();

  int getPlayerIndex();

  @TRADEService
  void setObservation(Object observation);

}
