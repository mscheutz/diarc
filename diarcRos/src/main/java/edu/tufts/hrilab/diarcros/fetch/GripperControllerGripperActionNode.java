/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.fetch;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RosException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class GripperControllerGripperActionNode {
  /**
   * Singleton instance.
   */
  private static GripperControllerGripperActionNode singleton;
  /**
   * Lock for singleton.
   */
  private static Lock singletonLock = new ReentrantLock();

  /**
   * Singleton accessor method. Thread-safe.
   *
   * @return
   */
  public static GripperControllerGripperActionNode getInstance() {
    singletonLock.lock();
    try {
      if (singleton == null) {
        singleton = new GripperControllerGripperActionNode();
      }
      return singleton;
    } finally {
      singletonLock.unlock();
    }
  }

  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Subscription local data & locks
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
  private final Object RosoutLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommand GripperControllerCommand;
  private final Object GripperControllerCommandLock = new Object();

  // Publishers
  private Publisher<control_msgs.JointControllerState> GripperControllerStatePublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  // Services
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> GripperControllerGripperActionNodeSetLoggerLevel;
  roscpp.SetLoggerLevelResponse GripperControllerGripperActionNodeSetLoggerLevelResponse;
  final Object GripperControllerGripperActionNodeSetLoggerLevelLock = new Object();
  boolean GripperControllerGripperActionNodeSetLoggerLevelCondition = false;
  boolean GripperControllerGripperActionNodeSetLoggerLevelSuccess = false;
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> GripperControllerGripperActionNodeGetLoggers;
  roscpp.GetLoggersResponse GripperControllerGripperActionNodeGetLoggersResponse;
  final Object GripperControllerGripperActionNodeGetLoggersLock = new Object();
  boolean GripperControllerGripperActionNodeGetLoggersCondition = false;
  boolean GripperControllerGripperActionNodeGetLoggersSuccess = false;
  // Action client
  private SimpleActionClient<
          control_msgs.GripperCommandActionFeedback,
          control_msgs.GripperCommandActionGoal,
          control_msgs.GripperCommandActionResult,
          control_msgs.GripperCommandFeedback,
          control_msgs.GripperCommandGoal,
          control_msgs.GripperCommandResult> client;

  private GripperControllerGripperActionNode() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/gripper_controller/gripper_action_node");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<rosgraph_msgs.Log> RosoutSub = node.newSubscriber("/rosout", rosgraph_msgs.Log._TYPE);
        RosoutSub.addMessageListener(new MessageListener<rosgraph_msgs.Log>() {
          @Override
          public void onNewMessage(rosgraph_msgs.Log msg) {
            synchronized (RosoutLock) {
              Rosout = edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log.toAde(msg);
            }
          }
        });
        Subscriber<control_msgs.GripperCommand> GripperControllerCommandSub = node.newSubscriber("/gripper_controller/command", control_msgs.GripperCommand._TYPE);
        GripperControllerCommandSub.addMessageListener(new MessageListener<control_msgs.GripperCommand>() {
          @Override
          public void onNewMessage(control_msgs.GripperCommand msg) {
            synchronized (GripperControllerCommandLock) {
              GripperControllerCommand = edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommand.toAde(msg);
            }
          }
        });
        // Publishers
        GripperControllerStatePublisher = node.newPublisher("/gripper_controller/state", control_msgs.JointControllerState._TYPE);
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        //Services
        try {
          GripperControllerGripperActionNodeSetLoggerLevel = node.newServiceClient("/gripper_controller/gripper_action_node/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          GripperControllerGripperActionNodeGetLoggers = node.newServiceClient("/gripper_controller/gripper_action_node/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        // Action Client
        try {
          client = new SimpleActionClient<
                  control_msgs.GripperCommandActionFeedback,
                  control_msgs.GripperCommandActionGoal,
                  control_msgs.GripperCommandActionResult,
                  control_msgs.GripperCommandFeedback,
                  control_msgs.GripperCommandGoal,
                  control_msgs.GripperCommandResult>("/gripper_controller/gripper_action",
                  new ActionSpec(control_msgs.GripperCommandAction.class,
                          "control_msgs/GripperCommandAction",
                          "control_msgs/GripperCommandActionFeedback",
                          "control_msgs/GripperCommandActionGoal",
                          "control_msgs/GripperCommandActionResult",
                          "control_msgs/GripperCommandFeedback",
                          "control_msgs/GripperCommandGoal",
                          "control_msgs/GripperCommandResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (client == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        client.addClientPubSub(node);

        // notify of node ready
        nodeReadyLock.lock();
        nodeReady = true;
        try {
          nodeReadyCond.signalAll();
        } finally {
          nodeReadyLock.unlock();
        }
      }
    };

    try {
      URI ros_master_uri = new URI(System.getenv("ROS_MASTER_URI"));
      String host = InetAddressFactory.newNonLoopback().getHostAddress();
      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, ros_master_uri);
      nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(nodeMain, nodeConfiguration);
    } catch (URISyntaxException e) {
      System.err.println("Error trying to create URI: " + e);
    }
  }

  // wait for node to be ready
  public void waitForNode() {
    nodeReadyLock.lock();
    try {
      while (!nodeReady) {
        nodeReadyCond.awaitUninterruptibly();
      }
    } finally {
      nodeReadyLock.unlock();
    }
  }

  // Subscribers
  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
    return Rosout;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommand getGripperControllerCommand() {
    return GripperControllerCommand;
  }

  // Publishers
  public void sendGripperControllerState(edu.tufts.hrilab.diarcros.msg.control_msgs.JointControllerState msg) {
    GripperControllerStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.control_msgs.JointControllerState.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  public void cancelAllGoals() {
    client.cancelAllGoals();
  }

  public void cancelGoal() throws RosException {
    client.cancelGoal();
  }

  public void cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    client.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public control_msgs.GripperCommandResult getResult() throws RosException {
    return client.getResult();
  }

  public SimpleClientGoalState getState() {
    return client.getState();
  }

  public void sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommandGoal goal) throws RosException {
    client.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommandGoal.toRos(goal, node));
  }


  public void waitForResult() throws InterruptedException {
    client.waitForResult();
  }

  public boolean waitForResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return client.waitForResult(timeout, units);
  }

  public void waitForServer() throws InterruptedException {
    client.waitForServer();
  }

  public boolean waitForServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return client.waitForServer(timeout, units);
  }
}

