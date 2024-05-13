/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import edu.tufts.hrilab.vision.stm.Grasp;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class GripperComponent extends DiarcComponent {
  protected GenericManipulator manip;
  protected String gripperClassName;

  @Override
  protected void init() {
    if (gripperClassName == null || gripperClassName.isEmpty()) {
      log.error("No gripper class specified.");
    } else {
      manip = GenericManipulator.instantiateGenericManipulator(gripperClassName);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("gripper").hasArg().argName("class").desc("Specify gripper class.").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("gripper")) {
      gripperClassName = cmdLine.getOptionValue("gripper");
    }
  }

  @TRADEService
  @Action
  public boolean moveGripper(float position) {
    return manip.moveGripper(position);
  }

  @TRADEService
  @Action
  public boolean moveGripper(String groupName, float position) {
    return manip.moveGripper(position);
  }

  @TRADEService
  @Action
  public boolean closeGripper(String groupName) {
    return moveGripper(0.0f);
  }

  @TRADEService
  @Action
  public boolean openGripper(String groupName) {
    return moveGripper(0.01f); // meters open
  }

  @TRADEService
  public float getGripperPosition(String groupName) {
    return 0;
  }

  //TODO:brad: should this really be here?
  @TRADEService
  @Action
  public int graspToDistance(Grasp grasp) {
    return manip.graspToDistance(grasp);
  }
}
