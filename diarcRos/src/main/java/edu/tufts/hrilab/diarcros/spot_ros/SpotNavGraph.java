/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.spot_ros;

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

public class SpotNavGraph {
    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();
    ServiceClient<spot_msgs.SetGraphNavMappingStateRequest, spot_msgs.SetGraphNavMappingStateResponse> SpotSetGraphNavMappingState;
    spot_msgs.SetGraphNavMappingStateResponse SpotSetGraphNavMappingStateResponse;
    final Object SpotSetGraphNavMappingStateLock = new Object();
    boolean SpotSetGraphNavMappingStateCondition = false;
    boolean SpotSetGraphNavMappingStateSuccess = false;
    ServiceClient<spot_msgs.ListGraphRequest, spot_msgs.ListGraphResponse> SpotListGraph;
    spot_msgs.ListGraphResponse SpotListGraphResponse;
    final Object SpotListGraphLock = new Object();
    boolean SpotListGraphCondition = false;
    boolean SpotListGraphSuccess = false;
    ServiceClient<spot_msgs.SetGraphNavMappingOperationRequest, spot_msgs.SetGraphNavMappingOperationResponse> SpotSetGraphNavMappingOperation;
    spot_msgs.SetGraphNavMappingOperationResponse SpotSetGraphNavMappingOperationResponse;
    final Object SpotSetGraphNavMappingOperationLock = new Object();
    boolean SpotSetGraphNavMappingOperationCondition = false;
    boolean SpotSetGraphNavMappingOperationSuccess = false;
    ServiceClient<spot_msgs.SetMapRequest, spot_msgs.SetMapResponse> SpotSetMap;
    spot_msgs.SetMapResponse SpotSetMapResponse;
    final Object SpotSetMapLock = new Object();
    boolean SpotSetMapCondition = false;
    boolean SpotSetMapSuccess = false;


    // Action clients
    /*
      SpotMsgsTrajectory
      spot_msgs/Trajectory
      spot_msgs.TrajectoryAction.class
      {:topic &quot;spot_msgs/TrajectoryAction&quot;, :type &quot;spot_msgs.TrajectoryAction&quot;}
      SpotMsgsFeedback
      spot_msgs/Feedback
      spot_msgs.FeedbackAction.class
      {:topic &quot;spot_msgs/FeedbackAction&quot;, :type &quot;spot_msgs.FeedbackAction&quot;}
      SpotMsgsPoseBody
      spot_msgs/PoseBody
      spot_msgs.PoseBodyAction.class
      {:topic &quot;spot_msgs/PoseBodyAction&quot;, :type &quot;spot_msgs.PoseBodyAction&quot;}
      SpotMsgsPoseBody
      spot_msgs/PoseBody
      spot_msgs.PoseBodyAction.class
      {:topic &quot;spot_msgs/PoseBodyAction&quot;, :type &quot;spot_msgs.PoseBodyAction&quot;}
      SpotMsgsDock
      spot_msgs/Dock
      spot_msgs.DockAction.class
      {:topic &quot;spot_msgs/DockAction&quot;, :type &quot;spot_msgs.DockAction&quot;}
      SpotMsgsNavigateTo
      spot_msgs/NavigateTo
      spot_msgs.NavigateToAction.class
      {:topic &quot;spot_msgs/NavigateToAction&quot;, :type &quot;spot_msgs.NavigateToAction&quot;}
    */
        private SimpleActionClient<
            spot_msgs.NavigateToActionFeedback,
            spot_msgs.NavigateToActionGoal,
            spot_msgs.NavigateToActionResult,
            spot_msgs.NavigateToFeedback,
            spot_msgs.NavigateToGoal,
            spot_msgs.NavigateToResult> SpotMsgsNavigateToClient;
        

    public SpotNavGraph() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/spot/spot_navgraph");
            }
                
            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                //Services
                try {
                    SpotSetGraphNavMappingState = node.newServiceClient("/spot/set_graph_nav_mapping_state", spot_msgs.SetGraphNavMappingState._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotSetGraphNavMappingOperation = node.newServiceClient("/spot/set_graph_nav_mapping_operation", spot_msgs.SetGraphNavMappingOperation._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotSetMap = node.newServiceClient("/spot/set_map", spot_msgs.SetMap._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                // Action Client
                try {
                    SpotMsgsNavigateToClient = new SimpleActionClient<>("/spot/navigate_to",
                            new ActionSpec(spot_msgs.NavigateToAction.class,
                            "spot_msgs/NavigateToAction",
                            "spot_msgs/NavigateToActionFeedback",
							"spot_msgs/NavigateToActionGoal",
							"spot_msgs/NavigateToActionResult",
							"spot_msgs/NavigateToFeedback",
							"spot_msgs/NavigateToGoal",
							"spot_msgs/NavigateToResult"));
                                                    
                    
                } catch (RosException e) {
                    e.printStackTrace();
                }
                while (SpotMsgsNavigateToClient == null)
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                       e.printStackTrace();
                }
                SpotMsgsNavigateToClient.addClientPubSub(node);
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

    public boolean callSpotSetGraphNavMappingState(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingStateRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingStateResponse response) {
        SpotSetGraphNavMappingStateCondition = false;
        SpotSetGraphNavMappingStateSuccess = false;
        SpotSetGraphNavMappingState.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingStateRequest.toRos(request, node), 
                new ServiceResponseListener<spot_msgs.SetGraphNavMappingStateResponse>() {

            @Override
            public void onSuccess(spot_msgs.SetGraphNavMappingStateResponse mt) {
                SpotSetGraphNavMappingStateResponse = mt;
                synchronized(SpotSetGraphNavMappingStateLock) {
                    SpotSetGraphNavMappingStateCondition = true;
                    SpotSetGraphNavMappingStateSuccess = true;
                    SpotSetGraphNavMappingStateLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(SpotSetGraphNavMappingStateLock) {
                    SpotSetGraphNavMappingStateCondition = true;
                    SpotSetGraphNavMappingStateSuccess = false;
                    SpotSetGraphNavMappingStateLock.notify();
                }
            }
        });
        
        synchronized(SpotSetGraphNavMappingStateLock) {
            while(!SpotSetGraphNavMappingStateCondition) {
                try {
                    SpotSetGraphNavMappingStateLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotSetGraphNavMappingStateSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingStateResponse.toAde(SpotSetGraphNavMappingStateResponse, response);
        }
        return SpotSetGraphNavMappingStateSuccess;
    }

    public boolean callSpotListGraph(edu.tufts.hrilab.diarcros.msg.spot_msgs.ListGraphRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.ListGraphResponse response) {
        SpotListGraphCondition = false;
        SpotListGraphSuccess = false;
        SpotListGraph.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.ListGraphRequest.toRos(request, node), 
                new ServiceResponseListener<spot_msgs.ListGraphResponse>() {

            @Override
            public void onSuccess(spot_msgs.ListGraphResponse mt) {
                SpotListGraphResponse = mt;
                synchronized(SpotListGraphLock) {
                    SpotListGraphCondition = true;
                    SpotListGraphSuccess = true;
                    SpotListGraphLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(SpotListGraphLock) {
                    SpotListGraphCondition = true;
                    SpotListGraphSuccess = false;
                    SpotListGraphLock.notify();
                }
            }
        });
        
        synchronized(SpotListGraphLock) {
            while(!SpotListGraphCondition) {
                try {
                    SpotListGraphLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotListGraphSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.ListGraphResponse.toAde(SpotListGraphResponse, response);
        }
        return SpotListGraphSuccess;
    }

    public boolean callSpotSetGraphNavMappingOperation(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingOperationRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingOperationResponse response) {
        SpotSetGraphNavMappingOperationCondition = false;
        SpotSetGraphNavMappingOperationSuccess = false;
        SpotSetGraphNavMappingOperation.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingOperationRequest.toRos(request, node), 
                new ServiceResponseListener<spot_msgs.SetGraphNavMappingOperationResponse>() {

            @Override
            public void onSuccess(spot_msgs.SetGraphNavMappingOperationResponse mt) {
                SpotSetGraphNavMappingOperationResponse = mt;
                synchronized(SpotSetGraphNavMappingOperationLock) {
                    SpotSetGraphNavMappingOperationCondition = true;
                    SpotSetGraphNavMappingOperationSuccess = true;
                    SpotSetGraphNavMappingOperationLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(SpotSetGraphNavMappingOperationLock) {
                    SpotSetGraphNavMappingOperationCondition = true;
                    SpotSetGraphNavMappingOperationSuccess = false;
                    SpotSetGraphNavMappingOperationLock.notify();
                }
            }
        });
        
        synchronized(SpotSetGraphNavMappingOperationLock) {
            while(!SpotSetGraphNavMappingOperationCondition) {
                try {
                    SpotSetGraphNavMappingOperationLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotSetGraphNavMappingOperationSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.SetGraphNavMappingOperationResponse.toAde(SpotSetGraphNavMappingOperationResponse, response);
        }
        return SpotSetGraphNavMappingOperationSuccess;
    }

    public void cancelAllSpotMsgsNavigateToGoals() {
        SpotMsgsNavigateToClient.cancelAllGoals();
    }
        
    public void cancelSpotMsgsNavigateToGoal() throws RosException {
        SpotMsgsNavigateToClient.cancelGoal();
    }

    public void cancelSpotMsgsNavigateToGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
        SpotMsgsNavigateToClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
    }

    public spot_msgs.NavigateToResult getSpotMsgsNavigateToResult() throws RosException {
        return SpotMsgsNavigateToClient.getResult();
    }

    public SimpleClientGoalState getSpotMsgsNavigateToState() {
        return SpotMsgsNavigateToClient.getState();
    }

    public void sendSpotMsgsNavigateToGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.NavigateToGoal goal) throws RosException {
         SpotMsgsNavigateToClient.sendGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.NavigateToGoal.toRos(goal, node));
    }

    public void waitForSpotMsgsNavigateToResult() throws InterruptedException {
        SpotMsgsNavigateToClient.waitForResult();
    }

    public boolean waitForSpotMsgsNavigateToResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return SpotMsgsNavigateToClient.waitForResult(timeout, units);
    }

    public void waitForSpotMsgsNavigateToServer() throws InterruptedException {
        SpotMsgsNavigateToClient.waitForServer();
    }

    public boolean waitForSpotMsgsNavigateToServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return SpotMsgsNavigateToClient.waitForServer(timeout, units);
    }
    public boolean callSpotSetMap(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetMapRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.SetMapResponse response) {
        SpotSetMapCondition = false;
        SpotSetMapSuccess = false;
        SpotSetMap.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetMapRequest.toRos(request, node),
                new ServiceResponseListener<spot_msgs.SetMapResponse>() {

                    @Override
                    public void onSuccess(spot_msgs.SetMapResponse mt) {
                        SpotSetMapResponse = mt;
                        synchronized(SpotSetMapLock) {
                            SpotSetMapCondition = true;
                            SpotSetMapSuccess = true;
                            SpotSetMapLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(SpotSetMapLock) {
                            SpotSetMapCondition = true;
                            SpotSetMapSuccess = false;
                            SpotSetMapLock.notify();
                        }
                    }
                });

        synchronized(SpotSetMapLock) {
            while(!SpotSetMapCondition) {
                try {
                    SpotSetMapLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotSetMapSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.SetMapResponse.toAde(SpotSetMapResponse, response);
        }
        return SpotSetMapSuccess;
    }
}

