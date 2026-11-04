/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator.robotiq;

import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import org.ros.address.InetAddressFactory;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Publisher;
import robotiq_2f_gripper_control.Robotiq2FGripper_robot_output;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.TimeUnit;

public class _2f85 extends GenericManipulator {
  protected ConnectedNode _ConnectedNode;
  protected Publisher<Robotiq2FGripper_robot_output> RobotiqPublisher;
  boolean Pubsub_isReady = false;

  // Giving these class scope means that they get set once and remain
  // constant unless we go out of our way to set them later (which is
  // how the Robotiq python implementation does it for their message
  // generation needs)
  private Byte position = 0;
  private Byte speed = 0;
  private Byte force = 0;
  private Byte active = 0;
  private Byte gto = 0; // I know this name is terrible, but it's what Robotiq uses and doesn't provide documentation to hint what it might be
  private Byte atr = 0; // as above

  public _2f85() {
    super();
    maxGraspWidthMeters = 1.0f;
    NodeMain RobotiqPubSub = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/ros_industrial/robotiq");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        // On start, find and seek out the Robotiq gripper node
        _ConnectedNode = connectedNode;
        GraphName gn = GraphName.of("/Robotiq2FGripperRobotOutput");
        RobotiqPublisher = connectedNode.newPublisher(gn, robotiq_2f_gripper_control.Robotiq2FGripper_robot_output._TYPE);
//        RobotiqSubscriber = connectNode.newSubscriber(gn, Robotiq2FGripper_robot_input._TYPE); //TODO: Add sub to check current states
        Pubsub_isReady = true;
      }
    };

    try {
      NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      URI rosMasterURI = new URI(System.getenv("ROS_MASTER_URI"));
      String host = InetAddressFactory.newNonLoopback().getHostAddress();
      NodeConfiguration nodeconfig = NodeConfiguration.newPublic(host, rosMasterURI);
      nodeMainExecutor.execute(RobotiqPubSub, nodeconfig); // Launch onStart() of the RobotiqPubSub
    } catch (URISyntaxException e) {
      log.error("Robotiq Component unable to create URI: " + e);
    }

    try {
      // Don't do anything until the pubsub is ready to go
      int pubsubCount = 0;
      while (!Pubsub_isReady) if (++pubsubCount < 5) TimeUnit.SECONDS.sleep(1);
      else break;
      if (Pubsub_isReady)
        log.info("Publisher is ready!");
      else
        log.error("After 5 attempts, I was unable to connect to Robotiq2FGripperRobotInput... Proceeding, but things might break later.");
    } catch (InterruptedException e) {
      log.error(e);
    }

    // Now that we're connected, we can start up the gripper.
    checkAndActivateGripper();
  }

  private Robotiq2FGripper_robot_output genMsg(int rACT, int rGTO, int rATR, int rPR, int rSP, int rFR) {
    robotiq_2f_gripper_control.Robotiq2FGripper_robot_output returnMe = RobotiqPublisher.newMessage();

    // -1 is a flag value to indicate that we shouldn't mess with that parameter
    if (rACT != -1)
      active = (byte) rACT;
    if (rGTO != -1)
      gto = (byte) rGTO;
    if (rATR != -1)
      atr = (byte) rATR;
    if (rPR != -1)
      position = (byte) rPR;
    if (rSP != -1)
      speed = (byte) rSP;
    if (rFR != -1)
      force = (byte) rFR;

    // Now we set message parameters to their global-scoped counterparts, ensuring things that don't
    // need to change don't, and things that we do change have been
    returnMe.setRACT(active);
    returnMe.setRGTO(gto);
    returnMe.setRATR(atr);
    returnMe.setRPR(position);
    returnMe.setRSP(speed);
    returnMe.setRFR(force);

    return returnMe;
  }


  private void checkAndActivateGripper() {
    if (!(active == 1)) { // Don't bother if we're already activated
      try {
        active = 1;
        gto = 1;
        atr = 0;
        position = Byte.MAX_VALUE;  // Move to maximum position
        speed = Byte.MAX_VALUE;     // Max speed
        force = Byte.MAX_VALUE;     // Max force (meaning lowest sensitivity to resistive force input)

        // Base parameters have been set, so generate that standard message
        robotiq_2f_gripper_control.Robotiq2FGripper_robot_output msg = genMsg(active, gto, atr, position, speed, force);
        RobotiqPublisher.publish(msg);

        // Standard activation message has been published, now put together parameters we more likely want for typical use
        msg = genMsg(-1, -1, -1, -1, 255, 150);
        RobotiqPublisher.publish(msg);
      } catch (RuntimeException e) {
        log.error(e);
        active = 0;
      }
    } /* TODO: else if(actual state is active when we didn't expect that), complain */
  }


  @Override
  public boolean moveGripper(float f) {
    checkAndActivateGripper();
    if (f > 1 || f < 0) {
      log.error("goToGripperPosition was given " + f + ": valid inputs are 0 to 1, where 0 is fully closed and 1 is fully open.");
      return false;
    }

    // Convert from what we want (0 is closed to 1 is open) to what Robotiq wants (255 is closed to 0 is open)
    f = ((1 - f) * 255);

    // And now send that to the publisher. We are truncating some data by converting to float, but it's negligible (and unavoidable)
    RobotiqPublisher.publish(genMsg(-1, -1, -1, (int) f, -1, -1)); // Set position but leave the rest alone

    try {
      // HACK: we're not actually checking against the real position, we're just waiting long enough to be done
      TimeUnit.SECONDS.sleep(2);
    } catch (Exception e) {
      log.warn(e);
    }
    return true; //TODO: use position state to return actual t/f
  }


  @Override
  public float getCurrentGripperPosition() {
    // TODO: put together that subscriber and get that actual data
    log.warn("Method not implemented. Returning 0.");
    return 0;
  }

  @Override
  public float getGoalGripperPosition() {
    // TODO: put together that subscriber and get that actual data
    return 0;
  }

  @Override
  public float getCurrentGripperEffort() {
    // TODO: put together that subscriber and get that actual data
    return 0;
  }

  @Override
  public void shutdown() {
    RobotiqPublisher.shutdown();
  }
}
