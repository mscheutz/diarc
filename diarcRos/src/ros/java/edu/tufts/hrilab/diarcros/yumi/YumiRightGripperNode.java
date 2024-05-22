/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.yumi;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

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

public class YumiRightGripperNode {

    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();

    ServiceClient<yumi_hw.YumiGraspRequest, yumi_hw.YumiGraspResponse> rightGripperCloseSrv;
    yumi_hw.YumiGraspResponse rightGripperCloseSrvResponse;
    final Object rightGripperCloseSrvLock = new Object();
    boolean rightGripperCloseSrvCondition = false;
    boolean rightGripperCloseSrvSuccess = false;
    ServiceClient<yumi_hw.YumiGraspRequest, yumi_hw.YumiGraspResponse> rightGripperOpenSrv;
    yumi_hw.YumiGraspResponse rightGripperOpenSrvResponse;
    final Object rightGripperOpenSrvLock = new Object();
    boolean rightGripperOpenSrvCondition = false;
    boolean rightGripperOpenSrvSuccess = false;
    public YumiRightGripperNode() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/yumi_right_gripper");
            }

            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                //Services
                try {
                    rightGripperCloseSrv = node.newServiceClient("/yumi/yumi_gripper/do_grasp", yumi_hw.YumiGrasp._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    rightGripperOpenSrv = node.newServiceClient("/yumi/yumi_gripper/release_grasp", yumi_hw.YumiGrasp._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
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
        while (!nodeReady) {
            try {
                nodeReadyCond.awaitUninterruptibly();
            } finally {
                nodeReadyLock.unlock();
            }
        }
    }

    public boolean callRightGripperCloseSrv(edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspRequest request, edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspResponse response) {
        rightGripperCloseSrvCondition = false;
        rightGripperCloseSrvSuccess = false;
        rightGripperCloseSrv.call(edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspRequest.toRos(request, node),
                new ServiceResponseListener<yumi_hw.YumiGraspResponse >() {

                    @Override
                    public void onSuccess(yumi_hw.YumiGraspResponse  mt) {
                        rightGripperCloseSrvResponse = mt;
                        synchronized(rightGripperCloseSrvLock) {
                            rightGripperCloseSrvCondition = true;
                            rightGripperCloseSrvSuccess = true;
                            rightGripperCloseSrvLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(rightGripperCloseSrvLock) {
                            rightGripperCloseSrvCondition = true;
                            rightGripperCloseSrvSuccess = false;
                            rightGripperCloseSrvLock.notify();
                        }
                    }
                });

        synchronized(rightGripperCloseSrvLock) {
            while(!rightGripperCloseSrvCondition) {
                try {
                    rightGripperCloseSrvLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (rightGripperCloseSrvSuccess) {
            edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspResponse.toAde(rightGripperCloseSrvResponse, response);
        }
        return rightGripperCloseSrvSuccess;
    }

    public boolean callRightGripperOpenSrv(edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspRequest request, edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspResponse response) {
        rightGripperOpenSrvCondition = false;
        rightGripperOpenSrvSuccess = false;
        rightGripperOpenSrv.call(edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspRequest.toRos(request, node),
                new ServiceResponseListener<yumi_hw.YumiGraspResponse >() {

                    @Override
                    public void onSuccess(yumi_hw.YumiGraspResponse  mt) {
                        rightGripperOpenSrvResponse = mt;
                        synchronized(rightGripperOpenSrvLock) {
                            rightGripperOpenSrvCondition = true;
                            rightGripperOpenSrvSuccess = true;
                            rightGripperOpenSrvLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(rightGripperOpenSrvLock) {
                            rightGripperOpenSrvCondition = true;
                            rightGripperOpenSrvSuccess = false;
                            rightGripperOpenSrvLock.notify();
                        }
                    }
                });

        synchronized(rightGripperOpenSrvLock) {
            while(!rightGripperOpenSrvCondition) {
                try {
                    rightGripperOpenSrvLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (rightGripperOpenSrvSuccess) {
            edu.tufts.hrilab.diarcros.msg.yumi_hw.YumiGraspResponse.toAde(rightGripperOpenSrvResponse, response);
        }
        return rightGripperOpenSrvSuccess;
    }
}
