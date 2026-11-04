/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.kortex;

import edu.tufts.hrilab.diarcros.moveit.MoveGroup;
import edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommand;
import edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommandGoal;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RosException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class GripperControllerGripperActionNode {
  /**
   * Singleton instance.
   */
  private static GripperControllerGripperActionNode singleton;
  /**
   * Lock for singleton.
   */
  private static Lock singletonLock = new ReentrantLock();

  private static Logger log = LoggerFactory.getLogger(GripperControllerGripperActionNode.class);

  // dynamic namespace and remappings
  private String namespace = "/";
  private Map<GraphName, GraphName> remappings = new HashMap<>();

  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Action client
  private SimpleActionClient<
          control_msgs.GripperCommandActionFeedback,
          control_msgs.GripperCommandActionGoal,
          control_msgs.GripperCommandActionResult,
          control_msgs.GripperCommandFeedback,
          control_msgs.GripperCommandGoal,
          control_msgs.GripperCommandResult> client;

  /**
   * Singleton accessor method. Thread-safe.
   * TODO: generalize this class to remove robotiq_2f_85 specific names.
   * TODO: allow the getInstance method to return different instances that have different namespace or remappings.
   * i.e., no longer an instance, but a factory method that manages existing instances and checks for matching
   * namespace and remappings.
   *
   * @return
   */
  public static GripperControllerGripperActionNode getInstance(String namespace, Map<String, String> remappings) {
    singletonLock.lock();
    try {
      if (singleton == null) {
        singleton = new GripperControllerGripperActionNode(namespace, remappings);
      } else if (!singleton.namespace.equals(namespace) || !remappingsMatch(remappings, singleton.remappings)) {
        log.error("Attempting to reassign the namespace or remappings of the existing DIARCROS node.");
        return null;
      }
      return singleton;
    } finally {
      singletonLock.unlock();
    }
  }

  /**
   * Helper method to check if String version and GraphName version of remappings are equivalent.
   *
   * @param first
   * @param second
   * @return
   */
  private static boolean remappingsMatch(Map<String, String> first, Map<GraphName, GraphName> second) {
    if ((first == null || first.isEmpty()) && (second == null || second.isEmpty())) {
      return true;
    } else if ((first == null || first.isEmpty()) || (second == null || second.isEmpty())) {
      return false;
    } else {
      Map<GraphName, GraphName> firstTmp = new HashMap<>();
      first.forEach((k, v) -> firstTmp.put(GraphName.of(k), GraphName.of(v)));
      return firstTmp.equals(second);
    }
  }

  private GripperControllerGripperActionNode(String namespace, Map<String, String> remappings) {
    this.namespace = namespace;
    remappings.forEach((k, v) -> this.remappings.put(GraphName.of(k), GraphName.of(v)));
    initialize();
  }

  private GripperControllerGripperActionNode() {
    initialize();
  }

  private void initialize() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/robotiq_2f_85_gripper_controller/gripper_action_node");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;

        // Action Client
        try {
          client = new SimpleActionClient<
                  control_msgs.GripperCommandActionFeedback,
                  control_msgs.GripperCommandActionGoal,
                  control_msgs.GripperCommandActionResult,
                  control_msgs.GripperCommandFeedback,
                  control_msgs.GripperCommandGoal,
                  control_msgs.GripperCommandResult>("robotiq_2f_85_gripper_controller/gripper_cmd",
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

        try {
          client.waitForServer(10, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
          log.error("Timed out waiting for action server to connect");
          throw new RuntimeException(e);
        }

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
      nodeConfiguration.setParentResolver(NameResolver.newFromNamespaceAndRemappings(namespace, remappings));
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

