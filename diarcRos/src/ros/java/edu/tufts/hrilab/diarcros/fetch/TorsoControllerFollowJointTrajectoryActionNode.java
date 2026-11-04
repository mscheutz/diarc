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
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class TorsoControllerFollowJointTrajectoryActionNode {
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
  private Publisher<control_msgs.JointTrajectoryControllerState> TorsoControllerStatePublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  // Action clients
  private SimpleActionClient<
          control_msgs.FollowJointTrajectoryActionFeedback,
          control_msgs.FollowJointTrajectoryActionGoal,
          control_msgs.FollowJointTrajectoryActionResult,
          control_msgs.FollowJointTrajectoryFeedback,
          control_msgs.FollowJointTrajectoryGoal,
          control_msgs.FollowJointTrajectoryResult> TorsoControlMsgsFollowJointTrajectoryClient;

  public TorsoControllerFollowJointTrajectoryActionNode() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/torso_controller/follow_joint_trajectory_action_node");
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
        TorsoControllerStatePublisher = node.newPublisher("/torso_controller/state", control_msgs.JointTrajectoryControllerState._TYPE);
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        //Services
        // Action Client
        try {
          TorsoControlMsgsFollowJointTrajectoryClient = new SimpleActionClient<>("/torso_controller/follow_joint_trajectory",
                  new ActionSpec(control_msgs.FollowJointTrajectoryAction.class,
                          "control_msgs/FollowJointTrajectoryAction",
                          "control_msgs/FollowJointTrajectoryActionFeedback",
                          "control_msgs/FollowJointTrajectoryActionGoal",
                          "control_msgs/FollowJointTrajectoryActionResult",
                          "control_msgs/FollowJointTrajectoryFeedback",
                          "control_msgs/FollowJointTrajectoryGoal",
                          "control_msgs/FollowJointTrajectoryResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (TorsoControlMsgsFollowJointTrajectoryClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        TorsoControlMsgsFollowJointTrajectoryClient.addClientPubSub(node);

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

  public synchronized edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory getTorsoControllerCommand() {
    return TorsoControllerCommand;
  }

  // Publishers
  public void sendTorsoControllerState(edu.tufts.hrilab.diarcros.msg.control_msgs.JointTrajectoryControllerState msg) {
    TorsoControllerStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.control_msgs.JointTrajectoryControllerState.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  // Action(s) Methods
  public void cancelAllTorsoControlMsgsFollowJointTrajectoryGoals() {
    TorsoControlMsgsFollowJointTrajectoryClient.cancelAllGoals();
  }

  public void cancelTorsoControlMsgsFollowJointTrajectoryGoal() throws RosException {
    TorsoControlMsgsFollowJointTrajectoryClient.cancelGoal();
  }

  public void cancelTorsoControlMsgsFollowJointTrajectoryGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    TorsoControlMsgsFollowJointTrajectoryClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public control_msgs.FollowJointTrajectoryResult getTorsoControlMsgsFollowJointTrajectoryResult() throws RosException {
    return TorsoControlMsgsFollowJointTrajectoryClient.getResult();
  }

  public SimpleClientGoalState getTorsoControlMsgsFollowJointTrajectoryState() {
    return TorsoControlMsgsFollowJointTrajectoryClient.getState();
  }

  public void sendTorsoControlMsgsFollowJointTrajectoryGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal goal) throws RosException {
    TorsoControlMsgsFollowJointTrajectoryClient.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal.toRos(goal, node));
  }

  public void waitForTorsoControlMsgsFollowJointTrajectoryResult() throws InterruptedException {
    TorsoControlMsgsFollowJointTrajectoryClient.waitForResult();
  }

  public boolean waitForTorsoControlMsgsFollowJointTrajectoryResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return TorsoControlMsgsFollowJointTrajectoryClient.waitForResult(timeout, units);
  }

  public void waitForTorsoControlMsgsFollowJointTrajectoryServer() throws InterruptedException {
    TorsoControlMsgsFollowJointTrajectoryClient.waitForServer();
  }

  public boolean waitForTorsoControlMsgsFollowJointTrajectoryServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return TorsoControlMsgsFollowJointTrajectoryClient.waitForServer(timeout, units);
  }
}

