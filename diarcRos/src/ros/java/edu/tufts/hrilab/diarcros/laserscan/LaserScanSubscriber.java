/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.laserscan;

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
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class LaserScanSubscriber {
    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();

    // Local data & Locks
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan BaseScan;
    private final Object BaseScanLock = new Object();

    // Publishers
    // Services
    ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> LaserScanSetLoggerLevel;
    roscpp.SetLoggerLevelResponse LaserScanSetLoggerLevelResponse;
    final Object LaserScanSetLoggerLevelLock = new Object();
    boolean LaserScanSetLoggerLevelCondition = false;
    boolean LaserScanSetLoggerLevelSuccess = false;
    ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> LaserScanGetLoggers;
    roscpp.GetLoggersResponse LaserScanGetLoggersResponse;
    final Object LaserScanGetLoggersLock = new Object();
    boolean LaserScanGetLoggersCondition = false;
    boolean LaserScanGetLoggersSuccess = false;
    public LaserScanSubscriber(String topic_name) {
        init(topic_name);
    }

    public LaserScanSubscriber() {
        init("/base_scan");
    }

    private void init(String topic_name) {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/laserscan");
            }

            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                // Subscribers
                Subscriber<sensor_msgs.LaserScan> BaseScanSub = node.newSubscriber(topic_name, sensor_msgs.LaserScan._TYPE);
                BaseScanSub.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
                    @Override
                    public void onNewMessage(sensor_msgs.LaserScan msg) {
                        synchronized (BaseScanLock) {
                            BaseScan = edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan.toAde(msg);
                        }
                    }
                });
                // Publishers
                //Services
                try {
                    LaserScanSetLoggerLevel = node.newServiceClient("/laserscan/set_logger_level", roscpp.SetLoggerLevel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    LaserScanGetLoggers = node.newServiceClient("/laserscan/get_loggers", roscpp.GetLoggers._TYPE);
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

    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan getScan() {
        return BaseScan;
    }
    // Services
    public boolean callLaserScanSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
        LaserScanSetLoggerLevelCondition = false;
        LaserScanSetLoggerLevelSuccess = false;
        LaserScanSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
                new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

                    @Override
                    public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                        LaserScanSetLoggerLevelResponse = mt;
                        synchronized(LaserScanSetLoggerLevelLock) {
                            LaserScanSetLoggerLevelCondition = true;
                            LaserScanSetLoggerLevelSuccess = true;
                            LaserScanSetLoggerLevelLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(LaserScanSetLoggerLevelLock) {
                            LaserScanSetLoggerLevelCondition = true;
                            LaserScanSetLoggerLevelSuccess = false;
                            LaserScanSetLoggerLevelLock.notify();
                        }
                    }
                });

        synchronized(LaserScanSetLoggerLevelLock) {
            while(!LaserScanSetLoggerLevelCondition) {
                try {
                    LaserScanSetLoggerLevelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (LaserScanSetLoggerLevelSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(LaserScanSetLoggerLevelResponse, response);
        }
        return LaserScanSetLoggerLevelSuccess;
    }
    public boolean callLaserScanGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
        LaserScanGetLoggersCondition = false;
        LaserScanGetLoggersSuccess = false;
        LaserScanGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
                new ServiceResponseListener<roscpp.GetLoggersResponse>() {

                    @Override
                    public void onSuccess(roscpp.GetLoggersResponse mt) {
                        LaserScanGetLoggersResponse = mt;
                        synchronized(LaserScanGetLoggersLock) {
                            LaserScanGetLoggersCondition = true;
                            LaserScanGetLoggersSuccess = true;
                            LaserScanGetLoggersLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(LaserScanGetLoggersLock) {
                            LaserScanGetLoggersCondition = true;
                            LaserScanGetLoggersSuccess = false;
                            LaserScanGetLoggersLock.notify();
                        }
                    }
                });

        synchronized(LaserScanGetLoggersLock) {
            while(!LaserScanGetLoggersCondition) {
                try {
                    LaserScanGetLoggersLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (LaserScanGetLoggersSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(LaserScanGetLoggersResponse, response);
        }
        return LaserScanGetLoggersSuccess;
    }
}
