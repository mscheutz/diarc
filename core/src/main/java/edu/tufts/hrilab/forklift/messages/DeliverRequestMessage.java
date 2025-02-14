/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift.messages;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;


//ros2 service call /controller_service crayler_nav_msgs/srv/ControllerCommand "{end_x: 2.0,
// end_y: 11.0, end_yaw: 1.5, driving_direction: 0, command: 0}"
@MessageMetadata(name = DeliverRequestMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class DeliverRequestMessage implements Message {
  static final String NAME = "crayler_nav_msgs/ControllerCommandServiceRequest";

  public String loading_area_id;
  public String unloading_area_id;

  public int nr_pallets=-1;

  public DeliverRequestMessage() {

  }
  public DeliverRequestMessage(String from, String to, int num) {
    loading_area_id = from;
    unloading_area_id = to;
    nr_pallets = num;
  }

  public DeliverRequestMessage(String from, String to) {
    loading_area_id = from;
    unloading_area_id = to;
    nr_pallets = 1;
  }
}

