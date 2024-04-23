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

public class Pr2MoveBaseNode {
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Publishers
  private Publisher<pr2_msgs.PowerState> PowerStatePublisher;
  private Publisher<std_msgs.Bool> NetworkConnectedPublisher;
  // Action clients
  private SimpleActionClient<
          move_base_msgs.MoveBaseActionFeedback,
          move_base_msgs.MoveBaseActionGoal,
          move_base_msgs.MoveBaseActionResult,
          move_base_msgs.MoveBaseFeedback,
          move_base_msgs.MoveBaseGoal,
          move_base_msgs.MoveBaseResult> MoveBaseMsgsMoveBaseClient;


  public Pr2MoveBaseNode() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/pr2_move_base_node");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;

        // Publishers
        PowerStatePublisher = node.newPublisher("/power_state", pr2_msgs.PowerState._TYPE);
        NetworkConnectedPublisher = node.newPublisher("/network/connected", std_msgs.Bool._TYPE);

        // Action Client
        try {
          MoveBaseMsgsMoveBaseClient = new SimpleActionClient<>("/move_base",
                  new ActionSpec(move_base_msgs.MoveBaseAction.class,
                          "move_base_msgs/MoveBaseAction",
                          "move_base_msgs/MoveBaseActionFeedback",
                          "move_base_msgs/MoveBaseActionGoal",
                          "move_base_msgs/MoveBaseActionResult",
                          "move_base_msgs/MoveBaseFeedback",
                          "move_base_msgs/MoveBaseGoal",
                          "move_base_msgs/MoveBaseResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (MoveBaseMsgsMoveBaseClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        MoveBaseMsgsMoveBaseClient.addClientPubSub(node);

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

  // Publishers
  public void sendPowerState(edu.tufts.hrilab.diarcros.msg.pr2_msgs.PowerState msg) {
    PowerStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.pr2_msgs.PowerState.toRos(msg, node));
  }

  public void sendNetworkConnected(edu.tufts.hrilab.diarcros.msg.std_msgs.Bool msg) {
    NetworkConnectedPublisher.publish(edu.tufts.hrilab.diarcros.msg.std_msgs.Bool.toRos(msg, node));
  }

  // Action(s) Methods
  public void cancelAllMoveBaseMsgsMoveBaseGoals() {
    MoveBaseMsgsMoveBaseClient.cancelAllGoals();
  }

  public void cancelMoveBaseMsgsMoveBaseGoal() throws RosException {
    MoveBaseMsgsMoveBaseClient.cancelGoal();
  }

  public void cancelMoveBaseMsgsMoveBaseGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    MoveBaseMsgsMoveBaseClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public move_base_msgs.MoveBaseResult getMoveBaseMsgsMoveBaseResult() throws RosException {
    return MoveBaseMsgsMoveBaseClient.getResult();
  }

  public SimpleClientGoalState getMoveBaseMsgsMoveBaseState() {
    return MoveBaseMsgsMoveBaseClient.getState();
  }

  public void sendMoveBaseMsgsMoveBaseGoal(edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal goal) throws RosException {
    MoveBaseMsgsMoveBaseClient.sendGoal(edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal.toRos(goal, node));
  }

  public void waitForMoveBaseMsgsMoveBaseResult() throws InterruptedException {
    MoveBaseMsgsMoveBaseClient.waitForResult();
  }

  public boolean waitForMoveBaseMsgsMoveBaseResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveBaseMsgsMoveBaseClient.waitForResult(timeout, units);
  }

  public void waitForMoveBaseMsgsMoveBaseServer() throws InterruptedException {
    MoveBaseMsgsMoveBaseClient.waitForServer();
  }

  public boolean waitForMoveBaseMsgsMoveBaseServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveBaseMsgsMoveBaseClient.waitForServer(timeout, units);
  }
}

