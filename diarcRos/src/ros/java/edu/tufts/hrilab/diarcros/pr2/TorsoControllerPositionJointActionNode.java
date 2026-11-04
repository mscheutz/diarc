/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.pr2;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.TimeUnit;
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

public class TorsoControllerPositionJointActionNode {
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
  private edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory TorsoControllerCommand;
  private final Object TorsoControllerCommandLock = new Object();

  // Publishers
  private Publisher<pr2_controllers_msgs.JointTrajectoryControllerState> TorsoControllerStatePublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  // Services
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> TorsoControllerPositionJointActionNodeGetLoggers;
  roscpp.GetLoggersResponse TorsoControllerPositionJointActionNodeGetLoggersResponse;
  final Object TorsoControllerPositionJointActionNodeGetLoggersLock = new Object();
  boolean TorsoControllerPositionJointActionNodeGetLoggersCondition = false;
  boolean TorsoControllerPositionJointActionNodeGetLoggersSuccess = false;
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> TorsoControllerPositionJointActionNodeSetLoggerLevel;
  roscpp.SetLoggerLevelResponse TorsoControllerPositionJointActionNodeSetLoggerLevelResponse;
  final Object TorsoControllerPositionJointActionNodeSetLoggerLevelLock = new Object();
  boolean TorsoControllerPositionJointActionNodeSetLoggerLevelCondition = false;
  boolean TorsoControllerPositionJointActionNodeSetLoggerLevelSuccess = false;
  // Action clients
  private SimpleActionClient<
          pr2_controllers_msgs.SingleJointPositionActionFeedback,
          pr2_controllers_msgs.SingleJointPositionActionGoal,
          pr2_controllers_msgs.SingleJointPositionActionResult,
          pr2_controllers_msgs.SingleJointPositionFeedback,
          pr2_controllers_msgs.SingleJointPositionGoal,
          pr2_controllers_msgs.SingleJointPositionResult> Pr2ControllersMsgsSingleJointPositionClient;


  public TorsoControllerPositionJointActionNode() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/torso_controller/position_joint_action_node");
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
        Subscriber<trajectory_msgs.JointTrajectory> TorsoControllerCommandSub = node.newSubscriber("/torso_controller/command", trajectory_msgs.JointTrajectory._TYPE);
        TorsoControllerCommandSub.addMessageListener(new MessageListener<trajectory_msgs.JointTrajectory>() {
          @Override
          public void onNewMessage(trajectory_msgs.JointTrajectory msg) {
            synchronized (TorsoControllerCommandLock) {
              TorsoControllerCommand = edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory.toAde(msg);
            }
          }
        });
        // Publishers
        TorsoControllerStatePublisher = node.newPublisher("/torso_controller/state", pr2_controllers_msgs.JointTrajectoryControllerState._TYPE);
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        //Services
        try {
          TorsoControllerPositionJointActionNodeGetLoggers = node.newServiceClient("/torso_controller/position_joint_action_node/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          TorsoControllerPositionJointActionNodeSetLoggerLevel = node.newServiceClient("/torso_controller/position_joint_action_node/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        // Action Client
        try {
          Pr2ControllersMsgsSingleJointPositionClient = new SimpleActionClient<>("/torso_controller/position_joint_action",
                  new ActionSpec(pr2_controllers_msgs.SingleJointPositionAction.class,
                          "pr2_controllers_msgs/SingleJointPositionAction",
                          "pr2_controllers_msgs/SingleJointPositionActionFeedback",
                          "pr2_controllers_msgs/SingleJointPositionActionGoal",
                          "pr2_controllers_msgs/SingleJointPositionActionResult",
                          "pr2_controllers_msgs/SingleJointPositionFeedback",
                          "pr2_controllers_msgs/SingleJointPositionGoal",
                          "pr2_controllers_msgs/SingleJointPositionResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (Pr2ControllersMsgsSingleJointPositionClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        Pr2ControllersMsgsSingleJointPositionClient.addClientPubSub(node);

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

  /**
   * Wait for node to be ready, with timeout. If timeout is reached and node is not ready, will return false.
   *
   * @param timeout how long to wait in seconds
   * @return
   */
  public boolean waitForNode(long timeout) {
    nodeReadyLock.lock();
    try {
      return nodeReadyCond.await(timeout, TimeUnit.SECONDS);
    } catch (InterruptedException e) {
      return false;
    } finally {
      nodeReadyLock.unlock();
    }
  }

  /**
   * Is node connected to ROS and ready.
   *
   * @return
   */
  public boolean isNodeReady() {
    nodeReadyLock.lock();
    try {
      return nodeReady;
    } finally {
      nodeReadyLock.unlock();
    }
  }

  // Subscribers
  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
    return Rosout;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory getTorsoControllerCommand() {
    return TorsoControllerCommand;
  }

  // Publishers
  public void sendTorsoControllerState(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState msg) {
    TorsoControllerStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  // Services
  public boolean callTorsoControllerPositionJointActionNodeGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
    TorsoControllerPositionJointActionNodeGetLoggersCondition = false;
    TorsoControllerPositionJointActionNodeGetLoggersSuccess = false;
    TorsoControllerPositionJointActionNodeGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.GetLoggersResponse>() {

              @Override
              public void onSuccess(roscpp.GetLoggersResponse mt) {
                TorsoControllerPositionJointActionNodeGetLoggersResponse = mt;
                synchronized (TorsoControllerPositionJointActionNodeGetLoggersLock) {
                  TorsoControllerPositionJointActionNodeGetLoggersCondition = true;
                  TorsoControllerPositionJointActionNodeGetLoggersSuccess = true;
                  TorsoControllerPositionJointActionNodeGetLoggersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (TorsoControllerPositionJointActionNodeGetLoggersLock) {
                  TorsoControllerPositionJointActionNodeGetLoggersCondition = true;
                  TorsoControllerPositionJointActionNodeGetLoggersSuccess = false;
                  TorsoControllerPositionJointActionNodeGetLoggersLock.notify();
                }
              }
            });

    synchronized (TorsoControllerPositionJointActionNodeGetLoggersLock) {
      while (!TorsoControllerPositionJointActionNodeGetLoggersCondition) {
        try {
          TorsoControllerPositionJointActionNodeGetLoggersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (TorsoControllerPositionJointActionNodeGetLoggersSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(TorsoControllerPositionJointActionNodeGetLoggersResponse, response);
    }
    return TorsoControllerPositionJointActionNodeGetLoggersSuccess;
  }

  public boolean callTorsoControllerPositionJointActionNodeSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
    TorsoControllerPositionJointActionNodeSetLoggerLevelCondition = false;
    TorsoControllerPositionJointActionNodeSetLoggerLevelSuccess = false;
    TorsoControllerPositionJointActionNodeSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

              @Override
              public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                TorsoControllerPositionJointActionNodeSetLoggerLevelResponse = mt;
                synchronized (TorsoControllerPositionJointActionNodeSetLoggerLevelLock) {
                  TorsoControllerPositionJointActionNodeSetLoggerLevelCondition = true;
                  TorsoControllerPositionJointActionNodeSetLoggerLevelSuccess = true;
                  TorsoControllerPositionJointActionNodeSetLoggerLevelLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (TorsoControllerPositionJointActionNodeSetLoggerLevelLock) {
                  TorsoControllerPositionJointActionNodeSetLoggerLevelCondition = true;
                  TorsoControllerPositionJointActionNodeSetLoggerLevelSuccess = false;
                  TorsoControllerPositionJointActionNodeSetLoggerLevelLock.notify();
                }
              }
            });

    synchronized (TorsoControllerPositionJointActionNodeSetLoggerLevelLock) {
      while (!TorsoControllerPositionJointActionNodeSetLoggerLevelCondition) {
        try {
          TorsoControllerPositionJointActionNodeSetLoggerLevelLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (TorsoControllerPositionJointActionNodeSetLoggerLevelSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(TorsoControllerPositionJointActionNodeSetLoggerLevelResponse, response);
    }
    return TorsoControllerPositionJointActionNodeSetLoggerLevelSuccess;
  }

  // Action(s) Methods
  public void cancelAllPr2ControllersMsgsSingleJointPositionGoals() {
    Pr2ControllersMsgsSingleJointPositionClient.cancelAllGoals();
  }

  public void cancelPr2ControllersMsgsSingleJointPositionGoal() throws RosException {
    Pr2ControllersMsgsSingleJointPositionClient.cancelGoal();
  }

  public void cancelPr2ControllersMsgsSingleJointPositionGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    Pr2ControllersMsgsSingleJointPositionClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public pr2_controllers_msgs.SingleJointPositionResult getPr2ControllersMsgsSingleJointPositionResult() throws RosException {
    return Pr2ControllersMsgsSingleJointPositionClient.getResult();
  }

  public SimpleClientGoalState getPr2ControllersMsgsSingleJointPositionState() {
    return Pr2ControllersMsgsSingleJointPositionClient.getState();
  }

  public void sendPr2ControllersMsgsSingleJointPositionGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.SingleJointPositionGoal goal) throws RosException {
    Pr2ControllersMsgsSingleJointPositionClient.sendGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.SingleJointPositionGoal.toRos(goal, node));
  }

  public void waitForPr2ControllersMsgsSingleJointPositionResult() throws InterruptedException {
    Pr2ControllersMsgsSingleJointPositionClient.waitForResult();
  }

  public boolean waitForPr2ControllersMsgsSingleJointPositionResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return Pr2ControllersMsgsSingleJointPositionClient.waitForResult(timeout, units);
  }

  public void waitForPr2ControllersMsgsSingleJointPositionServer() throws InterruptedException {
    Pr2ControllersMsgsSingleJointPositionClient.waitForServer();
  }

  public boolean waitForPr2ControllersMsgsSingleJointPositionServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return Pr2ControllersMsgsSingleJointPositionClient.waitForServer(timeout, units);
  }
}

