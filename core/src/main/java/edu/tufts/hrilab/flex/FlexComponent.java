/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.flex;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class FlexComponent extends DiarcComponent {
  /**
   * ip@port of hub container that python spokes should connect to
   */
  private String hubContainer;


  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("hubContainer").hasArg().argName("ip@port").desc("Set internal python modules to run as spoke with this hub").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("hubContainer")) {
      hubContainer = cmdLine.getOptionValue("hubContainer");
    }
  }

  @Override
  public void init() {
    PythonWrapper flex_wrapper = new PythonWrapper("diarc.flex.flex_wrapper", true, hubContainer);
    flex_wrapper.start();
  }

  @Action
  @TRADEService
  public Justification openDoor(Symbol refId) {
    // get pixel coordinates from refId

    MemoryObject mo = getMemoryObject(refId);
    if (mo == null) {
      return new ConditionJustification(false, Factory.createPredicate("found(object)"));
    }

    int x = mo.getBoundingBox().x;
    int y = mo.getBoundingBox().y;

    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("open_door_primitive").argTypes(int.class, int.class))
              .call(Justification.class, x, y);
    } catch (TRADEException e) {
      log.error("Error calling python open_door_primitive.", e);
    }

    return new ConditionJustification(false);
  }

  @Action
  @TRADEService
  public Justification pullOut(Symbol refId) {
    return openDoor(refId);
  }

  @Action
  @TRADEService
  public Justification graspObject(Symbol refId) {
    MemoryObject mo = getMemoryObject(refId);
    if (mo == null) {
      return new ConditionJustification(false, Factory.createPredicate("found(object)"));
    }

    int x = mo.getBoundingBox().x;
    int y = mo.getBoundingBox().y;

    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("grasp_object_primitive").argTypes(int.class, int.class))
              .call(Justification.class, x, y);
    } catch (TRADEException e) {
      log.error("Error calling python grasp_object_primitive.", e);
    }

    return new ConditionJustification(false);
  }

  @Action
  @TRADEService
  public Justification pushButton(Symbol refId) {
    MemoryObject mo = getMemoryObject(refId);
    if (mo == null) {
      return new ConditionJustification(false, Factory.createPredicate("found(object)"));
    }

    int x = mo.getBoundingBox().x;
    int y = mo.getBoundingBox().y;

    try {
      return TRADE.getAvailableService(new TRADEServiceConstraints().name("push_button_primitive").argTypes(int.class, int.class))
              .call(Justification.class, x, y);
    } catch (TRADEException e) {
      log.error("Error calling python push_button_primitive.", e);
    }

    return new ConditionJustification(false);
  }

  private MemoryObject getMemoryObject(Symbol refId) {
    MemoryObject mo = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      mo = tsi.call(MemoryObject.class, refId, MemoryObject.class);
    } catch (TRADEException e) {
      log.error("[moveTo] exception getting memory object from reference, returning null", e);
    }
    return mo;
  }


  public Justification openDoorTest() {
    int x = 1;
    int y = 2;

    Justification justification = null;
    try {
      justification = TRADE.getAvailableService(new TRADEServiceConstraints().name("open_door_primitive").argTypes(Integer.class, Integer.class))
              .call(Justification.class, x, y);
    } catch (TRADEException e) {
      log.error("Error calling python open_door_primitive.", e);
    }

    return justification;
  }
}
