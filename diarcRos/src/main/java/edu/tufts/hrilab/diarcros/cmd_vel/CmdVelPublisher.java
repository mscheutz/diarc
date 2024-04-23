/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.cmd_vel;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;
import org.ros.address.InetAddressFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

public class CmdVelPublisher {
    // ROS connection
    private ConnectedNode node;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();

    // Publishers
    private Publisher<geometry_msgs.Twist> CmdVelPublisher;

    public CmdVelPublisher() {
        // Publishers
        // notify of node ready
        NodeMain nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/cmd_vel");
            }

            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                // Publishers
                CmdVelPublisher = node.newPublisher("/cmd_vel", geometry_msgs.Twist._TYPE);
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
             NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
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

    public boolean isNodeReady() {
        return nodeReady;
    }

    public edu.tufts.hrilab.diarcros.msg.Time getCurrentTime() {
      return edu.tufts.hrilab.diarcros.msg.Time.toAde(node.getCurrentTime());
    }

    public void sendCmdVel(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist msg) {
        CmdVelPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist.toRos(msg, node));
    }

    public void sendLinearCmdVel(double x, double y, double z) {
        sendCmdVel(new Twist(
                new Vector3(x, y, z),
                new Vector3(0, 0, 0)
        ));
    }

    public void sendAngularCmdVel(double x, double y, double z){
        sendCmdVel(new Twist(
                new Vector3(0, 0, 0),
                new Vector3(x, y ,z)
        ));
    }

    public void sendStopCmdVel() {
        sendCmdVel(new Twist(
                new Vector3(0, 0, 0),
                new Vector3(0, 0, 0)
        ));
    }
}

