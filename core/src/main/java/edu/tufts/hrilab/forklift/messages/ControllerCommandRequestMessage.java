/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift.messages;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;


//ros2 service call /controller_service crayler_nav_msgs/srv/ControllerCommand "{end_x: 2.0,
// end_y: 11.0, end_yaw: 1.5, driving_direction: 0, command: 0}"
@MessageMetadata(name = ControllerCommandRequestMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class ControllerCommandRequestMessage implements Message {
  static final String NAME = "crayler_nav_msgs/ControllerCommandServiceRequest";

  public double end_x;
  public double end_y;
  public double end_yaw;

  public double v_desired;
  public double v_max;
  public double k;

  public boolean driving_direction;
  public byte command;

  public ControllerCommandRequestMessage() {

  }
  public ControllerCommandRequestMessage(double x, double y, double yaw) {
    end_x = x;
    end_y = y;
    end_yaw = yaw;
    driving_direction = false;
    command = 0;
  }
}

