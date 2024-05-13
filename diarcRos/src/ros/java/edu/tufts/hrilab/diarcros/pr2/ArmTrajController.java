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

public class ArmTrajController {
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Subscription local data & locks
  private edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState RArmControllerState;
  private final Object RArmControllerStateLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState LArmControllerState;
  private final Object LArmControllerStateLock = new Object();

  private SimpleActionClient<
          control_msgs.FollowJointTrajectoryActionFeedback,
          control_msgs.FollowJointTrajectoryActionGoal,
          control_msgs.FollowJointTrajectoryActionResult,
          control_msgs.FollowJointTrajectoryFeedback,
          control_msgs.FollowJointTrajectoryGoal,
          control_msgs.FollowJointTrajectoryResult> RArmControllerFollowJointTrajectoryClient;
  private SimpleActionClient<
          pr2_controllers_msgs.JointTrajectoryActionFeedback,
          pr2_controllers_msgs.JointTrajectoryActionGoal,
          pr2_controllers_msgs.JointTrajectoryActionResult,
          pr2_controllers_msgs.JointTrajectoryFeedback,
          pr2_controllers_msgs.JointTrajectoryGoal,
          pr2_controllers_msgs.JointTrajectoryResult> RArmControllerJointTrajectoryActionClient;
  private SimpleActionClient<
          control_msgs.FollowJointTrajectoryActionFeedback,
          control_msgs.FollowJointTrajectoryActionGoal,
          control_msgs.FollowJointTrajectoryActionResult,
          control_msgs.FollowJointTrajectoryFeedback,
          control_msgs.FollowJointTrajectoryGoal,
          control_msgs.FollowJointTrajectoryResult> LArmControllerFollowJointTrajectoryClient;
  private SimpleActionClient<
          pr2_controllers_msgs.JointTrajectoryActionFeedback,
          pr2_controllers_msgs.JointTrajectoryActionGoal,
          pr2_controllers_msgs.JointTrajectoryActionResult,
          pr2_controllers_msgs.JointTrajectoryFeedback,
          pr2_controllers_msgs.JointTrajectoryGoal,
          pr2_controllers_msgs.JointTrajectoryResult> LArmControllerJointTrajectoryActionClient;


  public ArmTrajController() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/arm_traj_controller");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;

        // Subscribers
        Subscriber<pr2_controllers_msgs.JointTrajectoryControllerState> RArmControllerStateSub = node.newSubscriber("/r_arm_controller/state", pr2_controllers_msgs.JointTrajectoryControllerState._TYPE);
        RArmControllerStateSub.addMessageListener(new MessageListener<pr2_controllers_msgs.JointTrajectoryControllerState>() {
          @Override
          public void onNewMessage(pr2_controllers_msgs.JointTrajectoryControllerState msg) {
            synchronized (RArmControllerStateLock) {
              RArmControllerState = edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState.toAde(msg);
            }
          }
        });
        Subscriber<pr2_controllers_msgs.JointTrajectoryControllerState> LArmControllerStateSub = node.newSubscriber("/l_arm_controller/state", pr2_controllers_msgs.JointTrajectoryControllerState._TYPE);
        LArmControllerStateSub.addMessageListener(new MessageListener<pr2_controllers_msgs.JointTrajectoryControllerState>() {
          @Override
          public void onNewMessage(pr2_controllers_msgs.JointTrajectoryControllerState msg) {
            synchronized (LArmControllerStateLock) {
              LArmControllerState = edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState.toAde(msg);
            }
          }
        });

        // Action Client(s)
        try {
          RArmControllerFollowJointTrajectoryClient = new SimpleActionClient<>("/r_arm_controller/follow_joint_trajectory",
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
        while (RArmControllerFollowJointTrajectoryClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        RArmControllerFollowJointTrajectoryClient.addClientPubSub(node);
        try {
          RArmControllerJointTrajectoryActionClient = new SimpleActionClient<>("/r_arm_controller/joint_trajectory_action",
                  new ActionSpec(pr2_controllers_msgs.JointTrajectoryAction.class,
                          "pr2_controllers_msgs/JointTrajectoryAction",
                          "pr2_controllers_msgs/JointTrajectoryActionFeedback",
                          "pr2_controllers_msgs/JointTrajectoryActionGoal",
                          "pr2_controllers_msgs/JointTrajectoryActionResult",
                          "pr2_controllers_msgs/JointTrajectoryFeedback",
                          "pr2_controllers_msgs/JointTrajectoryGoal",
                          "pr2_controllers_msgs/JointTrajectoryResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (RArmControllerJointTrajectoryActionClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        RArmControllerJointTrajectoryActionClient.addClientPubSub(node);
        try {
          LArmControllerFollowJointTrajectoryClient = new SimpleActionClient<>("/l_arm_controller/follow_joint_trajectory",
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
        while (LArmControllerFollowJointTrajectoryClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        LArmControllerFollowJointTrajectoryClient.addClientPubSub(node);
        try {
          LArmControllerJointTrajectoryActionClient = new SimpleActionClient<>("/l_arm_controller/joint_trajectory_action",
                  new ActionSpec(pr2_controllers_msgs.JointTrajectoryAction.class,
                          "pr2_controllers_msgs/JointTrajectoryAction",
                          "pr2_controllers_msgs/JointTrajectoryActionFeedback",
                          "pr2_controllers_msgs/JointTrajectoryActionGoal",
                          "pr2_controllers_msgs/JointTrajectoryActionResult",
                          "pr2_controllers_msgs/JointTrajectoryFeedback",
                          "pr2_controllers_msgs/JointTrajectoryGoal",
                          "pr2_controllers_msgs/JointTrajectoryResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (LArmControllerJointTrajectoryActionClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        LArmControllerJointTrajectoryActionClient.addClientPubSub(node);

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
      String host = InetAddressFactory.newNonLoopback().getHostAddress().toString();
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
  public synchronized edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState getRArmControllerState() {
    return RArmControllerState;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState getLArmControllerState() {
    return LArmControllerState;
  }

  // Action(s) Methods
  public void cancelAllRArmControllerFollowJointTrajectoryClientGoals() {
    RArmControllerFollowJointTrajectoryClient.cancelAllGoals();
  }

  public void cancelRArmControllerFollowJointTrajectoryClientGoal() throws RosException {
    RArmControllerFollowJointTrajectoryClient.cancelGoal();
  }

  public void cancelRArmControllerFollowJointTrajectoryClientGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    RArmControllerFollowJointTrajectoryClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public control_msgs.FollowJointTrajectoryResult getRArmControllerFollowJointTrajectoryClientResult() throws RosException {
    return RArmControllerFollowJointTrajectoryClient.getResult();
  }

  public SimpleClientGoalState getRArmControllerFollowJointTrajectoryClientState() {
    return RArmControllerFollowJointTrajectoryClient.getState();
  }

  public void sendRArmControllerFollowJointTrajectoryClientGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal goal) throws RosException {
    RArmControllerFollowJointTrajectoryClient.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal.toRos(goal, node));
  }

  public void waitForRArmControllerFollowJointTrajectoryClientResult() throws InterruptedException {
    RArmControllerFollowJointTrajectoryClient.waitForResult();
  }

  public boolean waitForRArmControllerFollowJointTrajectoryClientResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return RArmControllerFollowJointTrajectoryClient.waitForResult(timeout, units);
  }

  public void waitForRArmControllerFollowJointTrajectoryClientServer() throws InterruptedException {
    RArmControllerFollowJointTrajectoryClient.waitForServer();
  }

  public boolean waitForRArmControllerFollowJointTrajectoryClientServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return RArmControllerFollowJointTrajectoryClient.waitForServer(timeout, units);
  }

  public void cancelAllRArmControllerJointTrajectoryActionClientGoals() {
    RArmControllerJointTrajectoryActionClient.cancelAllGoals();
  }

  public void cancelRArmControllerJointTrajectoryActionClientGoal() throws RosException {
    RArmControllerJointTrajectoryActionClient.cancelGoal();
  }

  public void cancelRArmControllerJointTrajectoryActionClientGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    RArmControllerJointTrajectoryActionClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public pr2_controllers_msgs.JointTrajectoryResult getRArmControllerJointTrajectoryActionClientResult() throws RosException {
    return RArmControllerJointTrajectoryActionClient.getResult();
  }

  public SimpleClientGoalState getRArmControllerJointTrajectoryActionClientState() {
    return RArmControllerJointTrajectoryActionClient.getState();
  }

  public void sendRArmControllerJointTrajectoryActionClientGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryGoal goal) throws RosException {
    RArmControllerJointTrajectoryActionClient.sendGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryGoal.toRos(goal, node));
  }

  public void waitForRArmControllerJointTrajectoryActionClientResult() throws InterruptedException {
    RArmControllerJointTrajectoryActionClient.waitForResult();
  }

  public boolean waitForRArmControllerJointTrajectoryActionClientResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return RArmControllerJointTrajectoryActionClient.waitForResult(timeout, units);
  }

  public void waitForRArmControllerJointTrajectoryActionClientServer() throws InterruptedException {
    RArmControllerJointTrajectoryActionClient.waitForServer();
  }

  public boolean waitForRArmControllerJointTrajectoryActionClientServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return RArmControllerJointTrajectoryActionClient.waitForServer(timeout, units);
  }

  public void cancelAllLArmControllerFollowJointTrajectoryClientGoals() {
    LArmControllerFollowJointTrajectoryClient.cancelAllGoals();
  }

  public void cancelLArmControllerFollowJointTrajectoryClientGoal() throws RosException {
    LArmControllerFollowJointTrajectoryClient.cancelGoal();
  }

  public void cancelLArmControllerFollowJointTrajectoryClientGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    LArmControllerFollowJointTrajectoryClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public control_msgs.FollowJointTrajectoryResult getLArmControllerFollowJointTrajectoryClientResult() throws RosException {
    return LArmControllerFollowJointTrajectoryClient.getResult();
  }

  public SimpleClientGoalState getLArmControllerFollowJointTrajectoryClientState() {
    return LArmControllerFollowJointTrajectoryClient.getState();
  }

  public void sendLArmControllerFollowJointTrajectoryClientGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal goal) throws RosException {
    LArmControllerFollowJointTrajectoryClient.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal.toRos(goal, node));
  }

  public void waitForLArmControllerFollowJointTrajectoryClientResult() throws InterruptedException {
    LArmControllerFollowJointTrajectoryClient.waitForResult();
  }

  public boolean waitForLArmControllerFollowJointTrajectoryClientResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return LArmControllerFollowJointTrajectoryClient.waitForResult(timeout, units);
  }

  public void waitForLArmControllerFollowJointTrajectoryClientServer() throws InterruptedException {
    LArmControllerFollowJointTrajectoryClient.waitForServer();
  }

  public boolean waitForLArmControllerFollowJointTrajectoryClientServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return LArmControllerFollowJointTrajectoryClient.waitForServer(timeout, units);
  }

  public void cancelAllLArmControllerJointTrajectoryActionClientGoals() {
    LArmControllerJointTrajectoryActionClient.cancelAllGoals();
  }

  public void cancelLArmControllerJointTrajectoryActionClientGoal() throws RosException {
    LArmControllerJointTrajectoryActionClient.cancelGoal();
  }

  public void cancelLArmControllerJointTrajectoryActionClientGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    LArmControllerJointTrajectoryActionClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public pr2_controllers_msgs.JointTrajectoryResult getLArmControllerJointTrajectoryActionClientResult() throws RosException {
    return LArmControllerJointTrajectoryActionClient.getResult();
  }

  public SimpleClientGoalState getLArmControllerJointTrajectoryActionClientState() {
    return LArmControllerJointTrajectoryActionClient.getState();
  }

  public void sendLArmControllerJointTrajectoryActionClientGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryGoal goal) throws RosException {
    LArmControllerJointTrajectoryActionClient.sendGoal(edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryGoal.toRos(goal, node));
  }

  public void waitForLArmControllerJointTrajectoryActionClientResult() throws InterruptedException {
    LArmControllerJointTrajectoryActionClient.waitForResult();
  }

  public boolean waitForLArmControllerJointTrajectoryActionClientResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return LArmControllerJointTrajectoryActionClient.waitForResult(timeout, units);
  }

  public void waitForLArmControllerJointTrajectoryActionClientServer() throws InterruptedException {
    LArmControllerJointTrajectoryActionClient.waitForServer();
  }

  public boolean waitForLArmControllerJointTrajectoryActionClientServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return LArmControllerJointTrajectoryActionClient.waitForServer(timeout, units);
  }

  public edu.tufts.hrilab.diarcros.msg.Time getCurrentTime() {
    return edu.tufts.hrilab.diarcros.msg.Time.toAde(node.getCurrentTime());
  }
}

