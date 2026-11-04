/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.pr2;

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

public class RGripperControllerGripperActionNode {
  /**
   * Singleton instance.
   */
  private static RGripperControllerGripperActionNode singleton;
  /**
   * Lock for singleton.
   */
  private static Lock singletonLock = new ReentrantLock();

  /**
   * Singleton accessor method. Thread-safe.
   *
   * @return
   */
  public static RGripperControllerGripperActionNode getInstance() {
    singletonLock.lock();
    try {
      if (singleton == null) {
        singleton = new RGripperControllerGripperActionNode();
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
  private edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.Pr2GripperCommand RGripperControllerCommand;
  private final Object RGripperControllerCommandLock = new Object();

  // Publishers
  private Publisher<pr2_controllers_msgs.JointControllerState> RGripperControllerStatePublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  // Services
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> RGripperControllerGripperActionNodeSetLoggerLevel;
  roscpp.SetLoggerLevelResponse RGripperControllerGripperActionNodeSetLoggerLevelResponse;
  final Object RGripperControllerGripperActionNodeSetLoggerLevelLock = new Object();
  boolean RGripperControllerGripperActionNodeSetLoggerLevelCondition = false;
  boolean RGripperControllerGripperActionNodeSetLoggerLevelSuccess = false;
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> RGripperControllerGripperActionNodeGetLoggers;
  roscpp.GetLoggersResponse RGripperControllerGripperActionNodeGetLoggersResponse;
  final Object RGripperControllerGripperActionNodeGetLoggersLock = new Object();
  boolean RGripperControllerGripperActionNodeGetLoggersCondition = false;
  boolean RGripperControllerGripperActionNodeGetLoggersSuccess = false;
  // Action client
  private SimpleActionClient<
          pr2_controllers_msgs.Pr2GripperCommandActionFeedback,
          pr2_controllers_msgs.Pr2GripperCommandActionGoal,
          pr2_controllers_msgs.Pr2GripperCommandActionResult,
          pr2_controllers_msgs.Pr2GripperCommandFeedback,
          pr2_controllers_msgs.Pr2GripperCommandGoal,
          pr2_controllers_msgs.Pr2GripperCommandResult> client;

  private RGripperControllerGripperActionNode() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/r_gripper_controller/gripper_action_node");
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
        Subscriber<pr2_controllers_msgs.Pr2GripperCommand> RGripperControllerCommandSub = node.newSubscriber("/r_gripper_controller/command", pr2_controllers_msgs.Pr2GripperCommand._TYPE);
        RGripperControllerCommandSub.addMessageListener(new MessageListener<pr2_controllers_msgs.Pr2GripperCommand>() {
          @Override
          public void onNewMessage(pr2_controllers_msgs.Pr2GripperCommand msg) {
            synchronized (RGripperControllerCommandLock) {
              RGripperControllerCommand = edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.Pr2GripperCommand.toAde(msg);
            }
          }
        });
        // Publishers
        RGripperControllerStatePublisher = node.newPublisher("/r_gripper_controller/state", pr2_controllers_msgs.JointControllerState._TYPE);
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        //Services
        try {
          RGripperControllerGripperActionNodeSetLoggerLevel = node.newServiceClient("/r_gripper_controller/gripper_action_node/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          RGripperControllerGripperActionNodeGetLoggers = node.newServiceClient("/r_gripper_controller/gripper_action_node/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        // Action Client
        try {
          client = new SimpleActionClient<
                  pr2_controllers_msgs.Pr2GripperCommandActionFeedback,
                  pr2_controllers_msgs.Pr2GripperCommandActionGoal,
                  pr2_controllers_msgs.Pr2GripperCommandActionResult,
                  pr2_controllers_msgs.Pr2GripperCommandFeedback,
                  pr2_controllers_msgs.Pr2GripperCommandGoal,
                  pr2_controllers_msgs.Pr2GripperCommandResult>("/r_gripper_controller/gripper_action",
                  new ActionSpec(pr2_controllers_msgs.Pr2GripperCommandAction.class,
                          "pr2_controllers_msgs/Pr2GripperCommandAction",
                          "pr2_controllers_msgs/Pr2GripperCommandActionFeedback",
                          "pr2_controllers_msgs/Pr2GripperCommandActionGoal",
                          "pr2_controllers_msgs/Pr2GripperCommandActionResult",
                          "pr2_controllers_msgs/Pr2GripperCommandFeedback",
                          "pr2_controllers_msgs/Pr2GripperCommandGoal",
                          "pr2_controllers_msgs/Pr2GripperCommandResult"));


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

  public synchronized edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.Pr2GripperCommand getRGripperControllerCommand() {
    return RGripperControllerCommand;
  }

  // Publishers
  public void sendRGripperControllerState(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointControllerState msg) {
    RGripperControllerStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointControllerState.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  // Services
  public boolean callRGripperControllerGripperActionNodeSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
    RGripperControllerGripperActionNodeSetLoggerLevelCondition = false;
    RGripperControllerGripperActionNodeSetLoggerLevelSuccess = false;
    RGripperControllerGripperActionNodeSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

              @Override
              public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                RGripperControllerGripperActionNodeSetLoggerLevelResponse = mt;
                synchronized (RGripperControllerGripperActionNodeSetLoggerLevelLock) {
                  RGripperControllerGripperActionNodeSetLoggerLevelCondition = true;
                  RGripperControllerGripperActionNodeSetLoggerLevelSuccess = true;
                  RGripperControllerGripperActionNodeSetLoggerLevelLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (RGripperControllerGripperActionNodeSetLoggerLevelLock) {
                  RGripperControllerGripperActionNodeSetLoggerLevelCondition = true;
                  RGripperControllerGripperActionNodeSetLoggerLevelSuccess = false;
                  RGripperControllerGripperActionNodeSetLoggerLevelLock.notify();
                }
              }
            });

    synchronized (RGripperControllerGripperActionNodeSetLoggerLevelLock) {
      while (!RGripperControllerGripperActionNodeSetLoggerLevelCondition) {
        try {
          RGripperControllerGripperActionNodeSetLoggerLevelLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (RGripperControllerGripperActionNodeSetLoggerLevelSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(RGripperControllerGripperActionNodeSetLoggerLevelResponse, response);
    }
    return RGripperControllerGripperActionNodeSetLoggerLevelSuccess;
  }

  public boolean callRGripperControllerGripperActionNodeGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
    RGripperControllerGripperActionNodeGetLoggersCondition = false;
    RGripperControllerGripperActionNodeGetLoggersSuccess = false;
    RGripperControllerGripperActionNodeGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.GetLoggersResponse>() {

              @Override
              public void onSuccess(roscpp.GetLoggersResponse mt) {
                RGripperControllerGripperActionNodeGetLoggersResponse = mt;
                synchronized (RGripperControllerGripperActionNodeGetLoggersLock) {
                  RGripperControllerGripperActionNodeGetLoggersCondition = true;
                  RGripperControllerGripperActionNodeGetLoggersSuccess = true;
                  RGripperControllerGripperActionNodeGetLoggersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (RGripperControllerGripperActionNodeGetLoggersLock) {
                  RGripperControllerGripperActionNodeGetLoggersCondition = true;
                  RGripperControllerGripperActionNodeGetLoggersSuccess = false;
                  RGripperControllerGripperActionNodeGetLoggersLock.notify();
                }
              }
            });

    synchronized (RGripperControllerGripperActionNodeGetLoggersLock) {
      while (!RGripperControllerGripperActionNodeGetLoggersCondition) {
        try {
          RGripperControllerGripperActionNodeGetLoggersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (RGripperControllerGripperActionNodeGetLoggersSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(RGripperControllerGripperActionNodeGetLoggersResponse, response);
    }
    return RGripperControllerGripperActionNodeGetLoggersSuccess;
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

  public pr2_controllers_msgs.Pr2GripperCommandResult getResult() throws RosException {
    return client.getResult();
  }

  public SimpleClientGoalState getState() {
    return client.getState();
  }

  public void sendGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.Pr2GripperCommandGoal goal) throws RosException {
    client.sendGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.Pr2GripperCommandGoal.toRos(goal, node));
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

