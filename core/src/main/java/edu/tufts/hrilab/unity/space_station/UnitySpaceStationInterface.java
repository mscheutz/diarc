package edu.tufts.hrilab.unity.space_station;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;

import java.util.List;

public interface UnitySpaceStationInterface {

  /**
   * Gets the health of the space station as a float [0:100].
   */
  @TRADEService
  @Action
  float getSpaceStationHealth();

  /**
   * Gets a list of damaged tubes, optionally limited to the area.
   */
  @TRADEService
  @Action
  List<Symbol> getSpaceStationTubesDamaged();

  @TRADEService
  @Action
  List<Symbol> getSpaceStationTubesDamaged(Symbol area);

  /**
   * Gets a list of broken tubes, optionally limited to the area.
   */
  @TRADEService
  @Action
  List<Symbol> getSpaceStationTubesBroken();

  @TRADEService
  @Action
  List<Symbol> getSpaceStationTubesBroken(Symbol area);

  /**
   * Gets a list of off tubes, optionally limited to a area.
   */
  @TRADEService
  @Action
  List<Symbol> getSpaceStationTubesOff();

  @TRADEService
  @Action
  List<Symbol> getSpaceStationTubesOff(Symbol area);

  /**
   * Gets the health of a specific tube via name as a float [0:100].
   */
  @TRADEService
  @Action
  float getSpaceStationTubeHealth(Symbol tube);

  /**
   * Gets the damaged state of a tube, true if damaged.
   */
  @TRADEService
  @Action
  boolean getSpaceStationTubeDamaged(Symbol tube);

  /**
   * Gets the broken state of a tube, true if broken.
   */
  @TRADEService
  @Action
  boolean getSpaceStationTubeBroken(Symbol tube);

  /**
   * Gets the off state of a tube, true if off.
   */
  @TRADEService
  @Action
  boolean getSpaceStationTubeOff(Symbol tube);
}
