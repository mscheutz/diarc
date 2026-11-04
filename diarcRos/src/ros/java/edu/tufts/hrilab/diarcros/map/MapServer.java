/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.map;

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
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class MapServer {
    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();

    // Services
    ServiceClient<nav_msgs.LoadMapRequest, nav_msgs.LoadMapResponse> ChangeMap;
    nav_msgs.LoadMapResponse ChangeMapResponse;
    final Object ChangeMapLock = new Object();
    boolean ChangeMapCondition = false;
    boolean ChangeMapSuccess = false;
    public MapServer() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/map_server");
            }
                
            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                //Services
                try {
                    ChangeMap = node.newServiceClient("/change_map", nav_msgs.LoadMap._TYPE);
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

    // Services
    public boolean callChangeMap(edu.tufts.hrilab.diarcros.msg.nav_msgs.LoadMapRequest request, edu.tufts.hrilab.diarcros.msg.nav_msgs.LoadMapResponse response) {
        ChangeMapCondition = false;
        ChangeMapSuccess = false;
        ChangeMap.call(edu.tufts.hrilab.diarcros.msg.nav_msgs.LoadMapRequest.toRos(request, node), 
                new ServiceResponseListener<nav_msgs.LoadMapResponse>() {

            @Override
            public void onSuccess(nav_msgs.LoadMapResponse mt) {
                ChangeMapResponse = mt;
                synchronized(ChangeMapLock) {
                    ChangeMapCondition = true;
                    ChangeMapSuccess = true;
                    ChangeMapLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(ChangeMapLock) {
                    ChangeMapCondition = true;
                    ChangeMapSuccess = false;
                    ChangeMapLock.notify();
                }
            }
        });
        
        synchronized(ChangeMapLock) {
            while(!ChangeMapCondition) {
                try {
                    ChangeMapLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (ChangeMapSuccess) {
            edu.tufts.hrilab.diarcros.msg.nav_msgs.LoadMapResponse.toAde(ChangeMapResponse, response);
        }
        return ChangeMapSuccess;
    }
}
    
