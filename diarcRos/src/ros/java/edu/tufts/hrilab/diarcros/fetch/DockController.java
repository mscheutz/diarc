/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.fetch;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import fetch_auto_dock_msgs.DockGoal;
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

public class DockController {
    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();

    // Action clients
    private SimpleActionClient<
            fetch_auto_dock_msgs.DockActionFeedback,
            fetch_auto_dock_msgs.DockActionGoal,
            fetch_auto_dock_msgs.DockActionResult,
            fetch_auto_dock_msgs.DockFeedback,
            fetch_auto_dock_msgs.DockGoal,
            fetch_auto_dock_msgs.DockResult> DockClient;
    private SimpleActionClient<
            fetch_auto_dock_msgs.UndockActionFeedback,
            fetch_auto_dock_msgs.UndockActionGoal,
            fetch_auto_dock_msgs.UndockActionResult,
            fetch_auto_dock_msgs.UndockFeedback,
            fetch_auto_dock_msgs.UndockGoal,
            fetch_auto_dock_msgs.UndockResult> UndockClient;

    public DockController() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/dock_controller/dock_controller_node");
            }

            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;

                try {
                    DockClient = new SimpleActionClient<>("/dock",
                            new ActionSpec(fetch_auto_dock_msgs.DockAction.class,
                                    "fetch_auto_dock_msgs/DockAction",
                                    "fetch_auto_dock_msgs/DockActionFeedback",
                                    "fetch_auto_dock_msgs/DockActionGoal",
                                    "fetch_auto_dock_msgs/DockActionResult",
                                    "fetch_auto_dock_msgs/DockFeedback",
                                    "fetch_auto_dock_msgs/DockGoal",
                                    "fetch_auto_dock_msgs/DockResult"));
                    UndockClient = new SimpleActionClient<>("/undock",
                            new ActionSpec(fetch_auto_dock_msgs.DockAction.class,
                                    "fetch_auto_dock_msgs/UndockAction",
                                    "fetch_auto_dock_msgs/UndockActionFeedback",
                                    "fetch_auto_dock_msgs/UndockActionGoal",
                                    "fetch_auto_dock_msgs/UndockActionResult",
                                    "fetch_auto_dock_msgs/UndockFeedback",
                                    "fetch_auto_dock_msgs/UndockGoal",
                                    "fetch_auto_dock_msgs/UndockResult"));


                } catch (RosException e) {
                    e.printStackTrace();
                }

                try {
                    DockClient.waitForServer(10, TimeUnit.SECONDS);
                    UndockClient.waitForServer(10, TimeUnit.SECONDS);
                } catch (InterruptedException e) {
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
        DockClient.cancelAllGoals();
        UndockClient.cancelAllGoals();
    }

    public void waitForServer() {
        try {
            DockClient.waitForServer();
            UndockClient.waitForServer();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public boolean callDock() {
        try {
            edu.tufts.hrilab.diarcros.msg.fetch_auto_dock_msgs.DockGoal goal = new edu.tufts.hrilab.diarcros.msg.fetch_auto_dock_msgs.DockGoal();
            DockClient.sendGoal(edu.tufts.hrilab.diarcros.msg.fetch_auto_dock_msgs.DockGoal.toRos(goal, node));
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public boolean callUndock() {
        try {
            edu.tufts.hrilab.diarcros.msg.fetch_auto_dock_msgs.UndockGoal goal = new edu.tufts.hrilab.diarcros.msg.fetch_auto_dock_msgs.UndockGoal();
            UndockClient.sendGoal(edu.tufts.hrilab.diarcros.msg.fetch_auto_dock_msgs.UndockGoal.toRos(goal, node));
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

}

