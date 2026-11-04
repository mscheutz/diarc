/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.common;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.address.InetAddressFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

public class BaseControllerCommandPub {
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Publishers
  private Publisher<geometry_msgs.Twist> BaseControllerCommandPublisher;

  public BaseControllerCommandPub() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/base_controller/command");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Publishers
        BaseControllerCommandPublisher = node.newPublisher("/base_controller/command", geometry_msgs.Twist._TYPE);
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

  public edu.tufts.hrilab.diarcros.msg.Time getCurrentTime() {
    return edu.tufts.hrilab.diarcros.msg.Time.toAde(node.getCurrentTime());
  }

  // Publishers
  public void sendBaseControllerCommand(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist msg) {
    BaseControllerCommandPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist.toRos(msg, node));
  }
}

