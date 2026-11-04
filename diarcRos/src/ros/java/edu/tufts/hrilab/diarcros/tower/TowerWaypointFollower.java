/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.tower;

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

public class TowerWaypointFollower {
    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();

    // Local data & Locks
    private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
    private final Object RosoutLock = new Object();

    // Publishers
    // Services
    ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> TowerWaypointFollowerSetLoggerLevel;
    roscpp.SetLoggerLevelResponse TowerWaypointFollowerSetLoggerLevelResponse;
    final Object TowerWaypointFollowerSetLoggerLevelLock = new Object();
    boolean TowerWaypointFollowerSetLoggerLevelCondition = false;
    boolean TowerWaypointFollowerSetLoggerLevelSuccess = false;
    ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> TowerWaypointFollowerGetLoggers;
    roscpp.GetLoggersResponse TowerWaypointFollowerGetLoggersResponse;
    final Object TowerWaypointFollowerGetLoggersLock = new Object();
    boolean TowerWaypointFollowerGetLoggersCondition = false;
    boolean TowerWaypointFollowerGetLoggersSuccess = false;
    ServiceClient<tower_waypoint_follower.getPoseArrayRequest, tower_waypoint_follower.getPoseArrayResponse> TowerWaypointFollower;
    tower_waypoint_follower.getPoseArrayResponse TowerWaypointFollowerResponse;
    final Object TowerWaypointFollowerLock = new Object();
    boolean TowerWaypointFollowerCondition = false;
    boolean TowerWaypointFollowerSuccess = false;
    public TowerWaypointFollower() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/tower_waypoint_follower");
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
                // Publishers
                //Services
                try {
                    TowerWaypointFollowerSetLoggerLevel = node.newServiceClient("/tower_waypoint_follower/set_logger_level", roscpp.SetLoggerLevel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    TowerWaypointFollowerGetLoggers = node.newServiceClient("/tower_waypoint_follower/get_loggers", roscpp.GetLoggers._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    TowerWaypointFollower = node.newServiceClient("/tower_waypoint_follower", tower_waypoint_follower.getPoseArray._TYPE);
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
    
    public edu.tufts.hrilab.diarcros.msg.Time getCurrentTime() {
      return edu.tufts.hrilab.diarcros.msg.Time.toAde(node.getCurrentTime());
    }

    public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
        return Rosout;
    }
    // Services
    public boolean callTowerWaypointFollowerSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
        TowerWaypointFollowerSetLoggerLevelCondition = false;
        TowerWaypointFollowerSetLoggerLevelSuccess = false;
        TowerWaypointFollowerSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

            @Override
            public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                TowerWaypointFollowerSetLoggerLevelResponse = mt;
                synchronized(TowerWaypointFollowerSetLoggerLevelLock) {
                    TowerWaypointFollowerSetLoggerLevelCondition = true;
                    TowerWaypointFollowerSetLoggerLevelSuccess = true;
                    TowerWaypointFollowerSetLoggerLevelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(TowerWaypointFollowerSetLoggerLevelLock) {
                    TowerWaypointFollowerSetLoggerLevelCondition = true;
                    TowerWaypointFollowerSetLoggerLevelSuccess = false;
                    TowerWaypointFollowerSetLoggerLevelLock.notify();
                }
            }
        });
        
        synchronized(TowerWaypointFollowerSetLoggerLevelLock) {
            while(!TowerWaypointFollowerSetLoggerLevelCondition) {
                try {
                    TowerWaypointFollowerSetLoggerLevelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (TowerWaypointFollowerSetLoggerLevelSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(TowerWaypointFollowerSetLoggerLevelResponse, response);
        }
        return TowerWaypointFollowerSetLoggerLevelSuccess;
    }
    public boolean callTowerWaypointFollowerGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
        TowerWaypointFollowerGetLoggersCondition = false;
        TowerWaypointFollowerGetLoggersSuccess = false;
        TowerWaypointFollowerGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.GetLoggersResponse>() {

            @Override
            public void onSuccess(roscpp.GetLoggersResponse mt) {
                TowerWaypointFollowerGetLoggersResponse = mt;
                synchronized(TowerWaypointFollowerGetLoggersLock) {
                    TowerWaypointFollowerGetLoggersCondition = true;
                    TowerWaypointFollowerGetLoggersSuccess = true;
                    TowerWaypointFollowerGetLoggersLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(TowerWaypointFollowerGetLoggersLock) {
                    TowerWaypointFollowerGetLoggersCondition = true;
                    TowerWaypointFollowerGetLoggersSuccess = false;
                    TowerWaypointFollowerGetLoggersLock.notify();
                }
            }
        });
        
        synchronized(TowerWaypointFollowerGetLoggersLock) {
            while(!TowerWaypointFollowerGetLoggersCondition) {
                try {
                    TowerWaypointFollowerGetLoggersLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (TowerWaypointFollowerGetLoggersSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(TowerWaypointFollowerGetLoggersResponse, response);
        }
        return TowerWaypointFollowerGetLoggersSuccess;
    }
    public boolean callTowerWaypointFollower(edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayRequest request, edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayResponse response) {
        TowerWaypointFollowerCondition = false;
        TowerWaypointFollowerSuccess = false;
        TowerWaypointFollower.call(edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayRequest.toRos(request, node), 
                new ServiceResponseListener<tower_waypoint_follower.getPoseArrayResponse>() {

            @Override
            public void onSuccess(tower_waypoint_follower.getPoseArrayResponse mt) {
                TowerWaypointFollowerResponse = mt;
                synchronized(TowerWaypointFollowerLock) {
                    TowerWaypointFollowerCondition = true;
                    TowerWaypointFollowerSuccess = true;
                    TowerWaypointFollowerLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(TowerWaypointFollowerLock) {
                    TowerWaypointFollowerCondition = true;
                    TowerWaypointFollowerSuccess = false;
                    TowerWaypointFollowerLock.notify();
                }
            }
        });
        
        synchronized(TowerWaypointFollowerLock) {
            while(!TowerWaypointFollowerCondition) {
                try {
                    TowerWaypointFollowerLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (TowerWaypointFollowerSuccess) {
            edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayResponse.toAde(TowerWaypointFollowerResponse, response);
        }
        return TowerWaypointFollowerSuccess;
    }
}
    
