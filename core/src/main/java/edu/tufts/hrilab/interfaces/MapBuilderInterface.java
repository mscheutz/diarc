package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Symbol;

public interface MapBuilderInterface {

  @TRADEService
  void startMapping();

  @TRADEService
  void createWaypoint(Symbol name);

  @TRADEService
  void finishMapping();
}
