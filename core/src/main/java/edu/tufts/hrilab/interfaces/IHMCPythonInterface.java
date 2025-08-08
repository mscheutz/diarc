package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;

/**
 * Interface to be implemented by a Python module, not a Java DIARC component.
 */
public interface IHMCPythonInterface {
  @TRADEService
  @Action
  void walk();

  @TRADEService
  @Action
  Justification goto_primitive(String target_location);

  @TRADEService
  @Action
  Justification pickup_primitive(String target_object);

  @TRADEService
  @Action
  Justification push_door_primitive();

  @TRADEService
  @Action
  Justification pull_door_primitive();
}
