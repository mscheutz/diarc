/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.turtlesim;

import java.net.URI;
import java.net.URISyntaxException;
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

public class TurtleShape {
    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // Subscription local data & locks
    private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
    private final Object RosoutLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist Turtle1CmdVel;
    private final Object Turtle1CmdVelLock = new Object();

    // Publishers
    private Publisher<turtlesim.Pose> Turtle1PosePublisher;
    // Services
    ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> TurtleShapeSetLoggerLevel;
    roscpp.SetLoggerLevelResponse TurtleShapeSetLoggerLevelResponse;
    final Object TurtleShapeSetLoggerLevelLock = new Object();
    boolean TurtleShapeSetLoggerLevelCondition = false;
    boolean TurtleShapeSetLoggerLevelSuccess = false;
    ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> TurtleShapeGetLoggers;
    roscpp.GetLoggersResponse TurtleShapeGetLoggersResponse;
    final Object TurtleShapeGetLoggersLock = new Object();
    boolean TurtleShapeGetLoggersCondition = false;
    boolean TurtleShapeGetLoggersSuccess = false;
    // Action client
    private SimpleActionClient<
            turtle_actionlib.ShapeActionFeedback,
            turtle_actionlib.ShapeActionGoal,
            turtle_actionlib.ShapeActionResult,
            turtle_actionlib.ShapeFeedback,
            turtle_actionlib.ShapeGoal,
            turtle_actionlib.ShapeResult> client;

    public TurtleShape() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/turtle_shape");
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
                Subscriber<geometry_msgs.Twist> Turtle1CmdVelSub = node.newSubscriber("/turtle1/cmd_vel", geometry_msgs.Twist._TYPE);
                Turtle1CmdVelSub.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
                    @Override
                    public void onNewMessage(geometry_msgs.Twist msg) {
                        synchronized (Turtle1CmdVelLock) {
                            Turtle1CmdVel = edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist.toAde(msg);
                        }
                    }
                });
                // Publishers
                Turtle1PosePublisher = node.newPublisher("/turtle1/pose", turtlesim.Pose._TYPE);
                //Services
                try {
                    TurtleShapeSetLoggerLevel = node.newServiceClient("/turtle_shape/set_logger_level", roscpp.SetLoggerLevel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    TurtleShapeGetLoggers = node.newServiceClient("/turtle_shape/get_loggers", roscpp.GetLoggers._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                // Action Client
                try {
                    client = new SimpleActionClient<
                                    turtle_actionlib.ShapeActionFeedback,
                                    turtle_actionlib.ShapeActionGoal,
                                    turtle_actionlib.ShapeActionResult,
                                    turtle_actionlib.ShapeFeedback,
                                    turtle_actionlib.ShapeGoal,
                                    turtle_actionlib.ShapeResult>("/turtle_shape",
                                         new ActionSpec(turtle_actionlib.ShapeAction.class,
                                                        "turtle_actionlib/ShapeAction",
                                                        "turtle_actionlib/ShapeActionFeedback",
                                                        "turtle_actionlib/ShapeActionGoal",
                                                        "turtle_actionlib/ShapeActionResult",
                                                        "turtle_actionlib/ShapeFeedback",
                                                        "turtle_actionlib/ShapeGoal",
                                                        "turtle_actionlib/ShapeResult"));
                                                    
                    
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
            }
        };
        
        try {
          URI ros_master_uri = new URI(System.getenv("ROS_MASTER_URI"));
          String host = InetAddressFactory.newNonLoopback().getHostAddress().toString();
          NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, ros_master_uri);
          nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
          nodeMainExecutor.execute(nodeMain, nodeConfiguration);
        } catch (URISyntaxException e) {
          System.err.println("Error trying to create URI: " + e);
        }
    }

    // Subscribers
    public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
        return Rosout;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist getTurtle1CmdVel() {
        return Turtle1CmdVel;
    }
    // Publishers
    public void sendTurtle1Pose(edu.tufts.hrilab.diarcros.msg.turtlesim.Pose msg) {
        Turtle1PosePublisher.publish(edu.tufts.hrilab.diarcros.msg.turtlesim.Pose.toRos(msg, node));
    }
    // Services
    public boolean callTurtleShapeSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
        TurtleShapeSetLoggerLevelCondition = false;
        TurtleShapeSetLoggerLevelSuccess = false;
        TurtleShapeSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

            @Override
            public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                TurtleShapeSetLoggerLevelResponse = mt;
                synchronized(TurtleShapeSetLoggerLevelLock) {
                    TurtleShapeSetLoggerLevelCondition = true;
                    TurtleShapeSetLoggerLevelSuccess = true;
                    TurtleShapeSetLoggerLevelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(TurtleShapeSetLoggerLevelLock) {
                    TurtleShapeSetLoggerLevelCondition = true;
                    TurtleShapeSetLoggerLevelSuccess = false;
                    TurtleShapeSetLoggerLevelLock.notify();
                }
            }
        });
        
        synchronized(TurtleShapeSetLoggerLevelLock) {
            while(!TurtleShapeSetLoggerLevelCondition) {
                try {
                    TurtleShapeSetLoggerLevelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (TurtleShapeSetLoggerLevelSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(TurtleShapeSetLoggerLevelResponse, response);
        }
        return TurtleShapeSetLoggerLevelSuccess;
    }
    public boolean callTurtleShapeGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
        TurtleShapeGetLoggersCondition = false;
        TurtleShapeGetLoggersSuccess = false;
        TurtleShapeGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.GetLoggersResponse>() {

            @Override
            public void onSuccess(roscpp.GetLoggersResponse mt) {
                TurtleShapeGetLoggersResponse = mt;
                synchronized(TurtleShapeGetLoggersLock) {
                    TurtleShapeGetLoggersCondition = true;
                    TurtleShapeGetLoggersSuccess = true;
                    TurtleShapeGetLoggersLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(TurtleShapeGetLoggersLock) {
                    TurtleShapeGetLoggersCondition = true;
                    TurtleShapeGetLoggersSuccess = false;
                    TurtleShapeGetLoggersLock.notify();
                }
            }
        });
        
        synchronized(TurtleShapeGetLoggersLock) {
            while(!TurtleShapeGetLoggersCondition) {
                try {
                    TurtleShapeGetLoggersLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (TurtleShapeGetLoggersSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(TurtleShapeGetLoggersResponse, response);
        }
        return TurtleShapeGetLoggersSuccess;
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

    public turtle_actionlib.ShapeResult getResult() throws RosException {
        return client.getResult();
    }

    public SimpleClientGoalState getState() {
        return client.getState();
    }

    public void sendGoal(edu.tufts.hrilab.diarcros.msg.turtle_actionlib.ShapeGoal goal) throws RosException {
         client.sendGoal(edu.tufts.hrilab.diarcros.msg.turtle_actionlib.ShapeGoal.toRos(goal, node));
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

