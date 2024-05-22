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

public class HeadTrajControllerPointHeadAction {
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Subscription local data & locks
  private edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory HeadTrajControllerCommand;
  private final Object HeadTrajControllerCommandLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
  private final Object RosoutLock = new Object();

  // Publishers
  private Publisher<tf2_msgs.TFMessage> TfPublisher;
  private Publisher<control_msgs.JointTrajectoryControllerState> HeadTrajControllerStatePublisher;
  private Publisher<tf2_msgs.TFMessage> TfStaticPublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  // Services
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> HeadTrajControllerPointHeadActionSetLoggerLevel;
  roscpp.SetLoggerLevelResponse HeadTrajControllerPointHeadActionSetLoggerLevelResponse;
  final Object HeadTrajControllerPointHeadActionSetLoggerLevelLock = new Object();
  boolean HeadTrajControllerPointHeadActionSetLoggerLevelCondition = false;
  boolean HeadTrajControllerPointHeadActionSetLoggerLevelSuccess = false;
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> HeadTrajControllerPointHeadActionGetLoggers;
  roscpp.GetLoggersResponse HeadTrajControllerPointHeadActionGetLoggersResponse;
  final Object HeadTrajControllerPointHeadActionGetLoggersLock = new Object();
  boolean HeadTrajControllerPointHeadActionGetLoggersCondition = false;
  boolean HeadTrajControllerPointHeadActionGetLoggersSuccess = false;
  // Action client
  private SimpleActionClient<
          control_msgs.PointHeadActionFeedback,
          control_msgs.PointHeadActionGoal,
          control_msgs.PointHeadActionResult,
          control_msgs.PointHeadFeedback,
          control_msgs.PointHeadGoal,
          control_msgs.PointHeadResult> client;

  public HeadTrajControllerPointHeadAction() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/head_controller/point_head");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<trajectory_msgs.JointTrajectory> HeadTrajControllerCommandSub = node.newSubscriber("/head_controller/command", trajectory_msgs.JointTrajectory._TYPE);
        HeadTrajControllerCommandSub.addMessageListener(new MessageListener<trajectory_msgs.JointTrajectory>() {
          @Override
          public void onNewMessage(trajectory_msgs.JointTrajectory msg) {
            synchronized (HeadTrajControllerCommandLock) {
              HeadTrajControllerCommand = edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory.toAde(msg);
            }
          }
        });
        Subscriber<rosgraph_msgs.Log> RosoutSub = node.newSubscriber("/rosout", rosgraph_msgs.Log._TYPE);
        RosoutSub.addMessageListener(new MessageListener<rosgraph_msgs.Log>() {
          @Override
          public void onNewMessage(rosgraph_msgs.Log msg) {
            synchronized (RosoutLock) {
              Rosout = edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log.toAde(msg);
            }
          }
        });
        // Publishers
        TfPublisher = node.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);
        HeadTrajControllerStatePublisher = node.newPublisher("/head_controller/state", control_msgs.JointTrajectoryControllerState._TYPE);
        TfStaticPublisher = node.newPublisher("/tf_static", tf2_msgs.TFMessage._TYPE);
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        //Services
        try {
          HeadTrajControllerPointHeadActionSetLoggerLevel = node.newServiceClient("/head_controller/headaction/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          HeadTrajControllerPointHeadActionGetLoggers = node.newServiceClient("/head_controller/point_head/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        // Action Client
        try {
          client = new SimpleActionClient<>("/head_controller/point_head",
                  new ActionSpec(control_msgs.PointHeadAction.class,
                          "control_msgs/PointHeadAction",
                          "control_msgs/PointHeadActionFeedback",
                          "control_msgs/PointHeadActionGoal",
                          "control_msgs/PointHeadActionResult",
                          "control_msgs/PointHeadFeedback",
                          "control_msgs/PointHeadGoal",
                          "control_msgs/PointHeadResult"));


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
  public synchronized edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory getHeadTrajControllerCommand() {
    return HeadTrajControllerCommand;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
    return Rosout;
  }

  // Publishers
  public void sendTf(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
    TfPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
  }

  public void sendHeadTrajControllerState(edu.tufts.hrilab.diarcros.msg.control_msgs.JointTrajectoryControllerState msg) {
    HeadTrajControllerStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.control_msgs.JointTrajectoryControllerState.toRos(msg, node));
  }

  public void sendTfStatic(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
    TfStaticPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  // Services
  public boolean callHeadTrajControllerPointHeadActionSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
    HeadTrajControllerPointHeadActionSetLoggerLevelCondition = false;
    HeadTrajControllerPointHeadActionSetLoggerLevelSuccess = false;
    HeadTrajControllerPointHeadActionSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

              @Override
              public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                HeadTrajControllerPointHeadActionSetLoggerLevelResponse = mt;
                synchronized (HeadTrajControllerPointHeadActionSetLoggerLevelLock) {
                  HeadTrajControllerPointHeadActionSetLoggerLevelCondition = true;
                  HeadTrajControllerPointHeadActionSetLoggerLevelSuccess = true;
                  HeadTrajControllerPointHeadActionSetLoggerLevelLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (HeadTrajControllerPointHeadActionSetLoggerLevelLock) {
                  HeadTrajControllerPointHeadActionSetLoggerLevelCondition = true;
                  HeadTrajControllerPointHeadActionSetLoggerLevelSuccess = false;
                  HeadTrajControllerPointHeadActionSetLoggerLevelLock.notify();
                }
              }
            });

    synchronized (HeadTrajControllerPointHeadActionSetLoggerLevelLock) {
      while (!HeadTrajControllerPointHeadActionSetLoggerLevelCondition) {
        try {
          HeadTrajControllerPointHeadActionSetLoggerLevelLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (HeadTrajControllerPointHeadActionSetLoggerLevelSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(HeadTrajControllerPointHeadActionSetLoggerLevelResponse, response);
    }
    return HeadTrajControllerPointHeadActionSetLoggerLevelSuccess;
  }

  public boolean callHeadTrajControllerPointHeadActionGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
    HeadTrajControllerPointHeadActionGetLoggersCondition = false;
    HeadTrajControllerPointHeadActionGetLoggersSuccess = false;
    HeadTrajControllerPointHeadActionGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.GetLoggersResponse>() {

              @Override
              public void onSuccess(roscpp.GetLoggersResponse mt) {
                HeadTrajControllerPointHeadActionGetLoggersResponse = mt;
                synchronized (HeadTrajControllerPointHeadActionGetLoggersLock) {
                  HeadTrajControllerPointHeadActionGetLoggersCondition = true;
                  HeadTrajControllerPointHeadActionGetLoggersSuccess = true;
                  HeadTrajControllerPointHeadActionGetLoggersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (HeadTrajControllerPointHeadActionGetLoggersLock) {
                  HeadTrajControllerPointHeadActionGetLoggersCondition = true;
                  HeadTrajControllerPointHeadActionGetLoggersSuccess = false;
                  HeadTrajControllerPointHeadActionGetLoggersLock.notify();
                }
              }
            });

    synchronized (HeadTrajControllerPointHeadActionGetLoggersLock) {
      while (!HeadTrajControllerPointHeadActionGetLoggersCondition) {
        try {
          HeadTrajControllerPointHeadActionGetLoggersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (HeadTrajControllerPointHeadActionGetLoggersSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(HeadTrajControllerPointHeadActionGetLoggersResponse, response);
    }
    return HeadTrajControllerPointHeadActionGetLoggersSuccess;
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

  public control_msgs.PointHeadResult getResult() throws RosException {
    return client.getResult();
  }

  public SimpleClientGoalState getState() {
    return client.getState();
  }

  public void sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.PointHeadGoal goal) throws RosException {
    client.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.PointHeadGoal.toRos(goal, node));
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

