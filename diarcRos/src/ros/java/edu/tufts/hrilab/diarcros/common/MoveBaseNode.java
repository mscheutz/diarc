/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.common;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import geometry_msgs.Twist;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RemoteException;
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

public class MoveBaseNode {
  private static Logger log = LoggerFactory.getLogger(MoveBaseNode.class);
  RosConfiguration rc;

  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Local data & Locks
  private edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseActionGoal MoveBaseGoal;
  private final Object MoveBaseGoalLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped MoveBaseNodeCurrentGoal;
  private final Object MoveBaseNodeCurrentGoalLock = new Object();

  // Publishers
  private Publisher<geometry_msgs.PoseStamped> MoveBaseSimpleGoalPublisher;
  private Publisher<geometry_msgs.Twist> CmdVelocityPublisher;

  // Services
  ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> ClearCostmaps;
  std_srvs.EmptyResponse ClearCostmapResponse;
  final Object ClearCostmapLock = new Object();
  boolean ClearCostmapCondition = false;
  boolean ClearCostmapSuccess = false;


  // Action clients
  private SimpleActionClient<move_base_msgs.MoveBaseActionFeedback,
            move_base_msgs.MoveBaseActionGoal,
            move_base_msgs.MoveBaseActionResult,
            move_base_msgs.MoveBaseFeedback,
            move_base_msgs.MoveBaseGoal,
            move_base_msgs.MoveBaseResult> MoveBaseClient;

  public MoveBaseNode() {
      this(new RosConfiguration());
  }

  public MoveBaseNode(RosConfiguration rc) {
    this.rc = rc;
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of(rc.namespace +"/ade/move_base_node_"+ rc.uniqueID);
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<move_base_msgs.MoveBaseActionGoal> MoveBaseGoalSub = node.newSubscriber(rc.namespace +"/move_base/goal", move_base_msgs.MoveBaseActionGoal._TYPE);
        MoveBaseGoalSub.addMessageListener(new MessageListener<move_base_msgs.MoveBaseActionGoal>() {
          @Override
          public void onNewMessage(move_base_msgs.MoveBaseActionGoal msg) {
            synchronized (MoveBaseGoalLock) {
              MoveBaseGoal = edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseActionGoal.toAde(msg);
            }
          }
        });
        Subscriber<geometry_msgs.PoseStamped> MoveBaseNodeCurrentGoalSub = node.newSubscriber(rc.namespace +"/move_base_node/current_goal", geometry_msgs.PoseStamped._TYPE);
        MoveBaseNodeCurrentGoalSub.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
          @Override
          public void onNewMessage(geometry_msgs.PoseStamped msg) {
            synchronized (MoveBaseNodeCurrentGoalLock) {
              MoveBaseNodeCurrentGoal = edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped.toAde(msg);
            }
          }
        });
        // Publishers
        MoveBaseSimpleGoalPublisher = node.newPublisher(rc.namespace +"/move_base_simple/goal", geometry_msgs.PoseStamped._TYPE);
        CmdVelocityPublisher = node.newPublisher("/cmd_vel", Twist._TYPE);

        // Action Client
        try {
          MoveBaseClient = new SimpleActionClient<>(rc.namespace +"/move_base",
                  new ActionSpec(move_base_msgs.MoveBaseAction.class,
                          "move_base_msgs/MoveBaseAction",
                          "move_base_msgs/MoveBaseActionFeedback",
                          "move_base_msgs/MoveBaseActionGoal",
                          "move_base_msgs/MoveBaseActionResult",
                          "move_base_msgs/MoveBaseFeedback",
                          "move_base_msgs/MoveBaseGoal",
                          "move_base_msgs/MoveBaseResult"));

        } catch (RosException e) {
          e.printStackTrace();
        }

        try {
          ClearCostmaps = node.newServiceClient("/move_base/clear_costmaps", std_srvs.Empty._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          e.printStackTrace();
        }

        while (MoveBaseClient == null) {
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        MoveBaseClient.addClientPubSub(node);

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
    String host = InetAddressFactory.newNonLoopback().getHostAddress();
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, rc.rosMasterUri);
    nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
    nodeMainExecutor.execute(nodeMain, nodeConfiguration);
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

  public synchronized edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseActionGoal getMoveBaseGoal() {
    return MoveBaseGoal;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped getMoveBaseNodeCurrentGoal() {
    return MoveBaseNodeCurrentGoal;
  }

  public void sendMoveBaseSimpleGoal(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped msg) {
    MoveBaseSimpleGoalPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped.toRos(msg, node));
  }

  public void sendCmdVelocity(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist msg) {
    CmdVelocityPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist.toRos(msg, node));
  }

  // Action(s) Methods
  public void cancelAllMoveBaseGoals() {
      MoveBaseClient.cancelAllGoals();
  }

  public void cancelMoveBaseGoal() throws RosException {
      MoveBaseClient.cancelGoal();
  }

  public void cancelMoveBaseGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
      MoveBaseClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public move_base_msgs.MoveBaseResult getMoveBaseResult() throws RosException {
      return MoveBaseClient.getResult();
  }

  public SimpleClientGoalState getMoveBaseState() {
      return MoveBaseClient.getState();
  }

  public void sendMoveBaseGoal(edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal goal) throws RosException {
    log.debug("sending movebase goal");
    MoveBaseClient.sendGoal(edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal.toRos(goal, node));
    log.debug("finished sending movebase goal");
  }

  public void waitForMoveBaseResult() throws InterruptedException {
      MoveBaseClient.waitForResult();
  }

  public boolean waitForMoveBaseResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
      return MoveBaseClient.waitForResult(timeout, units);
  }

public void waitForMoveBaseServer() throws InterruptedException {
      MoveBaseClient.waitForServer();
  }

  public boolean waitForMoveBaseServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
      return MoveBaseClient.waitForServer(timeout, units);
  }

  public boolean callClearCostmaps(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
    ClearCostmapCondition = false;
    ClearCostmapSuccess = false;
    ClearCostmaps.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node),
            new ServiceResponseListener<std_srvs.EmptyResponse>() {
              @Override
              public void onSuccess(std_srvs.EmptyResponse emptyResponse) {
                ClearCostmapResponse = emptyResponse;
                synchronized (ClearCostmapLock) {
                  ClearCostmapCondition = true;
                  ClearCostmapSuccess = true;
                  ClearCostmapLock.notify();
                }
              }

              @Override
              public void onFailure(RemoteException e) {
                synchronized (ClearCostmapLock) {
                  ClearCostmapCondition = true;
                  ClearCostmapSuccess = false;
                  ClearCostmapLock.notify();
              }
            }
    });
    synchronized (ClearCostmapLock) {
      while (!ClearCostmapCondition) {
        try {
          ClearCostmapLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ClearCostmapSuccess) {
      edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(ClearCostmapResponse, response);
    }
    return ClearCostmapCondition;
  }
}
