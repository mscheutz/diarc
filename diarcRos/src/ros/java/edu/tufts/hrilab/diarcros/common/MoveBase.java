/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.common;

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

public class MoveBase {
  private RosConfiguration rc;
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Subscription local data & locks
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseGlobalCostmapInflaterParameterUpdates;
  private final Object MoveBaseGlobalCostmapInflaterParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud MoveBaseGlobalCostmapObstaclesClearingEndpoints;
  private final Object MoveBaseGlobalCostmapObstaclesClearingEndpointsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseParameterDescriptions;
  private final Object MoveBaseParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseGlobalCostmapStaticMapParameterUpdates;
  private final Object MoveBaseGlobalCostmapStaticMapParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
  private final Object RosoutLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseLocalCostmapObstaclesParameterDescriptions;
  private final Object MoveBaseLocalCostmapObstaclesParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseLocalCostmapInflaterParameterUpdates;
  private final Object MoveBaseLocalCostmapInflaterParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.map_msgs.OccupancyGridUpdate MoveBaseLocalCostmapCostmapUpdates;
  private final Object MoveBaseLocalCostmapCostmapUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped MoveBaseCurrentGoal;
  private final Object MoveBaseCurrentGoalLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseLocalCostmapObstaclesParameterUpdates;
  private final Object MoveBaseLocalCostmapObstaclesParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.nav_msgs.Path MoveBaseTrajectoryPlannerROSGlobalPlan;
  private final Object MoveBaseTrajectoryPlannerROSGlobalPlanLock = new Object();
  //private edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped MoveBaseGlobalCostmapFootprint;
  private final Object MoveBaseGlobalCostmapFootprintLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseParameterUpdates;
  private final Object MoveBaseParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.map_msgs.OccupancyGridUpdate MoveBaseGlobalCostmapCostmapUpdates;
  private final Object MoveBaseGlobalCostmapCostmapUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseLocalCostmapParameterDescriptions;
  private final Object MoveBaseLocalCostmapParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseGlobalCostmapParameterUpdates;
  private final Object MoveBaseGlobalCostmapParameterUpdatesLock = new Object();
  //private edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped MoveBaseLocalCostmapFootprint;
  private final Object MoveBaseLocalCostmapFootprintLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud MoveBaseLocalCostmapObstaclesClearingEndpoints;
  private final Object MoveBaseLocalCostmapObstaclesClearingEndpointsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseTrajectoryPlannerROSParameterUpdates;
  private final Object MoveBaseTrajectoryPlannerROSParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist CmdVel;
  private final Object CmdVelLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid MoveBaseLocalCostmapCostmap;
  private final Object MoveBaseLocalCostmapCostmapLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseGlobalCostmapStaticMapParameterDescriptions;
  private final Object MoveBaseGlobalCostmapStaticMapParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseGlobalCostmapInflaterParameterDescriptions;
  private final Object MoveBaseGlobalCostmapInflaterParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.nav_msgs.Path MoveBaseTrajectoryPlannerROSLocalPlan;
  private final Object MoveBaseTrajectoryPlannerROSLocalPlanLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseTrajectoryPlannerROSParameterDescriptions;
  private final Object MoveBaseTrajectoryPlannerROSParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseLocalCostmapParameterUpdates;
  private final Object MoveBaseLocalCostmapParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveBaseGlobalCostmapObstaclesParameterUpdates;
  private final Object MoveBaseGlobalCostmapObstaclesParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseLocalCostmapInflaterParameterDescriptions;
  private final Object MoveBaseLocalCostmapInflaterParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseGlobalCostmapObstaclesParameterDescriptions;
  private final Object MoveBaseGlobalCostmapObstaclesParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveBaseGlobalCostmapParameterDescriptions;
  private final Object MoveBaseGlobalCostmapParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud2 MoveBaseTrajectoryPlannerROSCostCloud;
  private final Object MoveBaseTrajectoryPlannerROSCostCloudLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.nav_msgs.Path MoveBaseNavfnROSPlan;
  private final Object MoveBaseNavfnROSPlanLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid MoveBaseGlobalCostmapCostmap;
  private final Object MoveBaseGlobalCostmapCostmapLock = new Object();

  // Publishers
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  private Publisher<nav_msgs.Odometry> OdomPublisher;
  private Publisher<sensor_msgs.LaserScan> BaseScanPublisher;
  //private Publisher<geometry_msgs.PolygonStamped> MoveBaseGlobalCostmapFootprintPublisher;
  private Publisher<nav_msgs.OccupancyGrid> MapPublisher;
  private Publisher<tf2_msgs.TFMessage> TfStaticPublisher;
  private Publisher<tf2_msgs.TFMessage> TfPublisher;
  private Publisher<geometry_msgs.PoseStamped> MoveBaseSimpleGoalPublisher;
  //private Publisher<geometry_msgs.PolygonStamped> MoveBaseLocalCostmapFootprintPublisher;
  private Publisher<sensor_msgs.CameraInfo> HeadCameraDepthDownsampleCameraInfoPublisher;
  private Publisher<sensor_msgs.Image> HeadCameraDepthDownsampleImageRawPublisher;
  // Services
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseSetParametersResponse;
  final Object MoveBaseSetParametersLock = new Object();
  boolean MoveBaseSetParametersCondition = false;
  boolean MoveBaseSetParametersSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseTrajectoryPlannerROSSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseTrajectoryPlannerROSSetParametersResponse;
  final Object MoveBaseTrajectoryPlannerROSSetParametersLock = new Object();
  boolean MoveBaseTrajectoryPlannerROSSetParametersCondition = false;
  boolean MoveBaseTrajectoryPlannerROSSetParametersSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseLocalCostmapSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseLocalCostmapSetParametersResponse;
  final Object MoveBaseLocalCostmapSetParametersLock = new Object();
  boolean MoveBaseLocalCostmapSetParametersCondition = false;
  boolean MoveBaseLocalCostmapSetParametersSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseLocalCostmapObstaclesSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseLocalCostmapObstaclesSetParametersResponse;
  final Object MoveBaseLocalCostmapObstaclesSetParametersLock = new Object();
  boolean MoveBaseLocalCostmapObstaclesSetParametersCondition = false;
  boolean MoveBaseLocalCostmapObstaclesSetParametersSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseLocalCostmapInflaterSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseLocalCostmapInflaterSetParametersResponse;
  final Object MoveBaseLocalCostmapInflaterSetParametersLock = new Object();
  boolean MoveBaseLocalCostmapInflaterSetParametersCondition = false;
  boolean MoveBaseLocalCostmapInflaterSetParametersSuccess = false;
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> MoveBaseGetLoggers;
  roscpp.GetLoggersResponse MoveBaseGetLoggersResponse;
  final Object MoveBaseGetLoggersLock = new Object();
  boolean MoveBaseGetLoggersCondition = false;
  boolean MoveBaseGetLoggersSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseGlobalCostmapStaticMapSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseGlobalCostmapStaticMapSetParametersResponse;
  final Object MoveBaseGlobalCostmapStaticMapSetParametersLock = new Object();
  boolean MoveBaseGlobalCostmapStaticMapSetParametersCondition = false;
  boolean MoveBaseGlobalCostmapStaticMapSetParametersSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseGlobalCostmapInflaterSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseGlobalCostmapInflaterSetParametersResponse;
  final Object MoveBaseGlobalCostmapInflaterSetParametersLock = new Object();
  boolean MoveBaseGlobalCostmapInflaterSetParametersCondition = false;
  boolean MoveBaseGlobalCostmapInflaterSetParametersSuccess = false;
  ServiceClient<nav_msgs.GetPlanRequest, nav_msgs.GetPlanResponse> MoveBaseMakePlan;
  nav_msgs.GetPlanResponse MoveBaseMakePlanResponse;
  final Object MoveBaseMakePlanLock = new Object();
  boolean MoveBaseMakePlanCondition = false;
  boolean MoveBaseMakePlanSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseGlobalCostmapObstaclesSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseGlobalCostmapObstaclesSetParametersResponse;
  final Object MoveBaseGlobalCostmapObstaclesSetParametersLock = new Object();
  boolean MoveBaseGlobalCostmapObstaclesSetParametersCondition = false;
  boolean MoveBaseGlobalCostmapObstaclesSetParametersSuccess = false;
  ServiceClient<nav_msgs.GetPlanRequest, nav_msgs.GetPlanResponse> MoveBaseNavfnROSMakePlan;
  nav_msgs.GetPlanResponse MoveBaseNavfnROSMakePlanResponse;
  final Object MoveBaseNavfnROSMakePlanLock = new Object();
  boolean MoveBaseNavfnROSMakePlanCondition = false;
  boolean MoveBaseNavfnROSMakePlanSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveBaseGlobalCostmapSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveBaseGlobalCostmapSetParametersResponse;
  final Object MoveBaseGlobalCostmapSetParametersLock = new Object();
  boolean MoveBaseGlobalCostmapSetParametersCondition = false;
  boolean MoveBaseGlobalCostmapSetParametersSuccess = false;
  ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> MoveBaseClearCostmaps;
  std_srvs.EmptyResponse MoveBaseClearCostmapsResponse;
  final Object MoveBaseClearCostmapsLock = new Object();
  boolean MoveBaseClearCostmapsCondition = false;
  boolean MoveBaseClearCostmapsSuccess = false;
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> MoveBaseSetLoggerLevel;
  roscpp.SetLoggerLevelResponse MoveBaseSetLoggerLevelResponse;
  final Object MoveBaseSetLoggerLevelLock = new Object();
  boolean MoveBaseSetLoggerLevelCondition = false;
  boolean MoveBaseSetLoggerLevelSuccess = false;
  // Action clients
    /*
      MoveBaseMsgsMoveBase
      move_base_msgs/MoveBase
      move_base_msgs.MoveBaseAction.class
      {:topic &quot;move_base_msgs/MoveBaseAction&quot;, :type &quot;move_base_msgs.MoveBaseAction&quot;}
    */

  private SimpleActionClient<
          move_base_msgs.MoveBaseActionFeedback,
          move_base_msgs.MoveBaseActionGoal,
          move_base_msgs.MoveBaseActionResult,
          move_base_msgs.MoveBaseFeedback,
          move_base_msgs.MoveBaseGoal,
          move_base_msgs.MoveBaseResult> MoveBaseMsgsMoveBaseClient;

  public MoveBase() {
      this(new RosConfiguration());
  }

  public MoveBase(RosConfiguration rc) {
    this.rc = rc;
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of(rc.namespace +"/diarc/move_base_"+ rc.uniqueID);
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<dynamic_reconfigure.Config> MoveBaseGlobalCostmapInflaterParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/inflater/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseGlobalCostmapInflaterParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseGlobalCostmapInflaterParameterUpdatesLock) {
              MoveBaseGlobalCostmapInflaterParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.PointCloud> MoveBaseGlobalCostmapObstaclesClearingEndpointsSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/obstacles/clearing_endpoints", sensor_msgs.PointCloud._TYPE);
        MoveBaseGlobalCostmapObstaclesClearingEndpointsSub.addMessageListener(new MessageListener<sensor_msgs.PointCloud>() {
          @Override
          public void onNewMessage(sensor_msgs.PointCloud msg) {
            synchronized (MoveBaseGlobalCostmapObstaclesClearingEndpointsLock) {
              MoveBaseGlobalCostmapObstaclesClearingEndpoints = edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseParameterDescriptionsLock) {
              MoveBaseParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveBaseGlobalCostmapStaticMapParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/static_map/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseGlobalCostmapStaticMapParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseGlobalCostmapStaticMapParameterUpdatesLock) {
              MoveBaseGlobalCostmapStaticMapParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<rosgraph_msgs.Log> RosoutSub = node.newSubscriber("/rosout", rosgraph_msgs.Log._TYPE);
        RosoutSub.addMessageListener(new MessageListener<rosgraph_msgs.Log>() {
          @Override
          public void onNewMessage(rosgraph_msgs.Log msg) {
            synchronized (RosoutLock) {
              Rosout = edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseLocalCostmapObstaclesParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/obstacles/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseLocalCostmapObstaclesParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseLocalCostmapObstaclesParameterDescriptionsLock) {
              MoveBaseLocalCostmapObstaclesParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveBaseLocalCostmapInflaterParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/inflater/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseLocalCostmapInflaterParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseLocalCostmapInflaterParameterUpdatesLock) {
              MoveBaseLocalCostmapInflaterParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<map_msgs.OccupancyGridUpdate> MoveBaseLocalCostmapCostmapUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/costmap_updates", map_msgs.OccupancyGridUpdate._TYPE);
        MoveBaseLocalCostmapCostmapUpdatesSub.addMessageListener(new MessageListener<map_msgs.OccupancyGridUpdate>() {
          @Override
          public void onNewMessage(map_msgs.OccupancyGridUpdate msg) {
            synchronized (MoveBaseLocalCostmapCostmapUpdatesLock) {
              MoveBaseLocalCostmapCostmapUpdates = edu.tufts.hrilab.diarcros.msg.map_msgs.OccupancyGridUpdate.toAde(msg);
            }
          }
        });
        Subscriber<geometry_msgs.PoseStamped> MoveBaseCurrentGoalSub = node.newSubscriber(rc.namespace +"/move_base/current_goal", geometry_msgs.PoseStamped._TYPE);
        MoveBaseCurrentGoalSub.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
          @Override
          public void onNewMessage(geometry_msgs.PoseStamped msg) {
            synchronized (MoveBaseCurrentGoalLock) {
              MoveBaseCurrentGoal = edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveBaseLocalCostmapObstaclesParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/obstacles/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseLocalCostmapObstaclesParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseLocalCostmapObstaclesParameterUpdatesLock) {
              MoveBaseLocalCostmapObstaclesParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<nav_msgs.Path> MoveBaseTrajectoryPlannerROSGlobalPlanSub = node.newSubscriber(rc.namespace +"/move_base/TrajectoryPlannerROS/global_plan", nav_msgs.Path._TYPE);
        MoveBaseTrajectoryPlannerROSGlobalPlanSub.addMessageListener(new MessageListener<nav_msgs.Path>() {
          @Override
          public void onNewMessage(nav_msgs.Path msg) {
            synchronized (MoveBaseTrajectoryPlannerROSGlobalPlanLock) {
              MoveBaseTrajectoryPlannerROSGlobalPlan = edu.tufts.hrilab.diarcros.msg.nav_msgs.Path.toAde(msg);
            }
          }
        });
        //Subscriber<geometry_msgs.PolygonStamped> MoveBaseGlobalCostmapFootprintSub = node.newSubscriber("/move_base/global_costmap/footprint", geometry_msgs.PolygonStamped._TYPE);
        //MoveBaseGlobalCostmapFootprintSub.addMessageListener(new MessageListener<geometry_msgs.PolygonStamped>() {
        //    @Override
        //    public void onNewMessage(geometry_msgs.PolygonStamped msg) {
        //        synchronized (MoveBaseGlobalCostmapFootprintLock) {
        //            MoveBaseGlobalCostmapFootprint = edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped.toAde(msg);
        //        }
        //    }
        //});
        Subscriber<dynamic_reconfigure.Config> MoveBaseParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseParameterUpdatesLock) {
              MoveBaseParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<map_msgs.OccupancyGridUpdate> MoveBaseGlobalCostmapCostmapUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/costmap_updates", map_msgs.OccupancyGridUpdate._TYPE);
        MoveBaseGlobalCostmapCostmapUpdatesSub.addMessageListener(new MessageListener<map_msgs.OccupancyGridUpdate>() {
          @Override
          public void onNewMessage(map_msgs.OccupancyGridUpdate msg) {
            synchronized (MoveBaseGlobalCostmapCostmapUpdatesLock) {
              MoveBaseGlobalCostmapCostmapUpdates = edu.tufts.hrilab.diarcros.msg.map_msgs.OccupancyGridUpdate.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseLocalCostmapParameterDescriptionsSub = node.newSubscriber("/move_base/local_costmap/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseLocalCostmapParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseLocalCostmapParameterDescriptionsLock) {
              MoveBaseLocalCostmapParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveBaseGlobalCostmapParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseGlobalCostmapParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseGlobalCostmapParameterUpdatesLock) {
              MoveBaseGlobalCostmapParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        //Subscriber<geometry_msgs.PolygonStamped> MoveBaseLocalCostmapFootprintSub = node.newSubscriber("/move_base/local_costmap/footprint", geometry_msgs.PolygonStamped._TYPE);
        //MoveBaseLocalCostmapFootprintSub.addMessageListener(new MessageListener<geometry_msgs.PolygonStamped>() {
        //    @Override
        //    public void onNewMessage(geometry_msgs.PolygonStamped msg) {
        //        synchronized (MoveBaseLocalCostmapFootprintLock) {
        //            MoveBaseLocalCostmapFootprint = edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped.toAde(msg);
        //        }
        //    }
        //});
        Subscriber<sensor_msgs.PointCloud> MoveBaseLocalCostmapObstaclesClearingEndpointsSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/obstacles/clearing_endpoints", sensor_msgs.PointCloud._TYPE);
        MoveBaseLocalCostmapObstaclesClearingEndpointsSub.addMessageListener(new MessageListener<sensor_msgs.PointCloud>() {
          @Override
          public void onNewMessage(sensor_msgs.PointCloud msg) {
            synchronized (MoveBaseLocalCostmapObstaclesClearingEndpointsLock) {
              MoveBaseLocalCostmapObstaclesClearingEndpoints = edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveBaseTrajectoryPlannerROSParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/TrajectoryPlannerROS/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseTrajectoryPlannerROSParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseTrajectoryPlannerROSParameterUpdatesLock) {
              MoveBaseTrajectoryPlannerROSParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<geometry_msgs.Twist> CmdVelSub = node.newSubscriber(rc.namespace +"/cmd_vel", geometry_msgs.Twist._TYPE);
        CmdVelSub.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
          @Override
          public void onNewMessage(geometry_msgs.Twist msg) {
            synchronized (CmdVelLock) {
              CmdVel = edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist.toAde(msg);
            }
          }
        });
        Subscriber<nav_msgs.OccupancyGrid> MoveBaseLocalCostmapCostmapSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/costmap", nav_msgs.OccupancyGrid._TYPE);
        MoveBaseLocalCostmapCostmapSub.addMessageListener(new MessageListener<nav_msgs.OccupancyGrid>() {
          @Override
          public void onNewMessage(nav_msgs.OccupancyGrid msg) {
            synchronized (MoveBaseLocalCostmapCostmapLock) {
              MoveBaseLocalCostmapCostmap = edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseGlobalCostmapStaticMapParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/static_map/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseGlobalCostmapStaticMapParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseGlobalCostmapStaticMapParameterDescriptionsLock) {
              MoveBaseGlobalCostmapStaticMapParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseGlobalCostmapInflaterParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/inflater/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseGlobalCostmapInflaterParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseGlobalCostmapInflaterParameterDescriptionsLock) {
              MoveBaseGlobalCostmapInflaterParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<nav_msgs.Path> MoveBaseTrajectoryPlannerROSLocalPlanSub = node.newSubscriber(rc.namespace +"/move_base/TrajectoryPlannerROS/local_plan", nav_msgs.Path._TYPE);
        MoveBaseTrajectoryPlannerROSLocalPlanSub.addMessageListener(new MessageListener<nav_msgs.Path>() {
          @Override
          public void onNewMessage(nav_msgs.Path msg) {
            synchronized (MoveBaseTrajectoryPlannerROSLocalPlanLock) {
              MoveBaseTrajectoryPlannerROSLocalPlan = edu.tufts.hrilab.diarcros.msg.nav_msgs.Path.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseTrajectoryPlannerROSParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/TrajectoryPlannerROS/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseTrajectoryPlannerROSParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseTrajectoryPlannerROSParameterDescriptionsLock) {
              MoveBaseTrajectoryPlannerROSParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveBaseLocalCostmapParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseLocalCostmapParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseLocalCostmapParameterUpdatesLock) {
              MoveBaseLocalCostmapParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveBaseGlobalCostmapObstaclesParameterUpdatesSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/obstacles/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveBaseGlobalCostmapObstaclesParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveBaseGlobalCostmapObstaclesParameterUpdatesLock) {
              MoveBaseGlobalCostmapObstaclesParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseLocalCostmapInflaterParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/local_costmap/inflater/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseLocalCostmapInflaterParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseLocalCostmapInflaterParameterDescriptionsLock) {
              MoveBaseLocalCostmapInflaterParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseGlobalCostmapObstaclesParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/obstacles/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseGlobalCostmapObstaclesParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseGlobalCostmapObstaclesParameterDescriptionsLock) {
              MoveBaseGlobalCostmapObstaclesParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveBaseGlobalCostmapParameterDescriptionsSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveBaseGlobalCostmapParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveBaseGlobalCostmapParameterDescriptionsLock) {
              MoveBaseGlobalCostmapParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.PointCloud2> MoveBaseTrajectoryPlannerROSCostCloudSub = node.newSubscriber(rc.namespace +"/move_base/TrajectoryPlannerROS/cost_cloud", sensor_msgs.PointCloud2._TYPE);
        MoveBaseTrajectoryPlannerROSCostCloudSub.addMessageListener(new MessageListener<sensor_msgs.PointCloud2>() {
          @Override
          public void onNewMessage(sensor_msgs.PointCloud2 msg) {
            synchronized (MoveBaseTrajectoryPlannerROSCostCloudLock) {
              MoveBaseTrajectoryPlannerROSCostCloud = edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud2.toAde(msg);
            }
          }
        });
        Subscriber<nav_msgs.Path> MoveBaseNavfnROSPlanSub = node.newSubscriber(rc.namespace +"/move_base/NavfnROS/plan", nav_msgs.Path._TYPE);
        MoveBaseNavfnROSPlanSub.addMessageListener(new MessageListener<nav_msgs.Path>() {
          @Override
          public void onNewMessage(nav_msgs.Path msg) {
            synchronized (MoveBaseNavfnROSPlanLock) {
              MoveBaseNavfnROSPlan = edu.tufts.hrilab.diarcros.msg.nav_msgs.Path.toAde(msg);
            }
          }
        });
        Subscriber<nav_msgs.OccupancyGrid> MoveBaseGlobalCostmapCostmapSub = node.newSubscriber(rc.namespace +"/move_base/global_costmap/costmap", nav_msgs.OccupancyGrid._TYPE);
        MoveBaseGlobalCostmapCostmapSub.addMessageListener(new MessageListener<nav_msgs.OccupancyGrid>() {
          @Override
          public void onNewMessage(nav_msgs.OccupancyGrid msg) {
            synchronized (MoveBaseGlobalCostmapCostmapLock) {
              MoveBaseGlobalCostmapCostmap = edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid.toAde(msg);
            }
          }
        });
        // Publishers
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        OdomPublisher = node.newPublisher(rc.namespace +"/odom", nav_msgs.Odometry._TYPE);
        BaseScanPublisher = node.newPublisher(rc.namespace +"/base_scan", sensor_msgs.LaserScan._TYPE);
        //MoveBaseGlobalCostmapFootprintPublisher = node.newPublisher("/move_base/global_costmap/footprint", geometry_msgs.PolygonStamped._TYPE);
        MapPublisher = node.newPublisher(rc.namespace +"/map", nav_msgs.OccupancyGrid._TYPE);
        TfStaticPublisher = node.newPublisher("/tf_static", tf2_msgs.TFMessage._TYPE);
        TfPublisher = node.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);
        MoveBaseSimpleGoalPublisher = node.newPublisher(rc.namespace +"/move_base_simple/goal", geometry_msgs.PoseStamped._TYPE);
        //MoveBaseLocalCostmapFootprintPublisher = node.newPublisher("/move_base/local_costmap/footprint", geometry_msgs.PolygonStamped._TYPE);
        HeadCameraDepthDownsampleCameraInfoPublisher = node.newPublisher(rc.namespace +"/head_camera/depth_downsample/camera_info", sensor_msgs.CameraInfo._TYPE);
        HeadCameraDepthDownsampleImageRawPublisher = node.newPublisher(rc.namespace +"/head_camera/depth_downsample/image_raw", sensor_msgs.Image._TYPE);
        //Services
        try {
          MoveBaseSetParameters = node.newServiceClient(rc.namespace +"/move_base/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseTrajectoryPlannerROSSetParameters = node.newServiceClient(rc.namespace +"/move_base/TrajectoryPlannerROS/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseLocalCostmapSetParameters = node.newServiceClient(rc.namespace +"/move_base/local_costmap/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseLocalCostmapObstaclesSetParameters = node.newServiceClient(rc.namespace +"/move_base/local_costmap/obstacles/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseLocalCostmapInflaterSetParameters = node.newServiceClient(rc.namespace +"/move_base/local_costmap/inflater/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseGetLoggers = node.newServiceClient(rc.namespace +"/move_base/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseGlobalCostmapStaticMapSetParameters = node.newServiceClient(rc.namespace +"/move_base/global_costmap/static_map/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseGlobalCostmapInflaterSetParameters = node.newServiceClient(rc.namespace +"/move_base/global_costmap/inflater/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseMakePlan = node.newServiceClient(rc.namespace +"/move_base/make_plan", nav_msgs.GetPlan._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseGlobalCostmapObstaclesSetParameters = node.newServiceClient(rc.namespace +"/move_base/global_costmap/obstacles/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseNavfnROSMakePlan = node.newServiceClient(rc.namespace +"/move_base/NavfnROS/make_plan", nav_msgs.GetPlan._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseGlobalCostmapSetParameters = node.newServiceClient(rc.namespace +"/move_base/global_costmap/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseClearCostmaps = node.newServiceClient(rc.namespace +"/move_base/clear_costmaps", std_srvs.Empty._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveBaseSetLoggerLevel = node.newServiceClient(rc.namespace +"/move_base/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        // Action Client
        try {
          MoveBaseMsgsMoveBaseClient = new SimpleActionClient<>(rc.namespace +"/move_base",
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
        while (MoveBaseMsgsMoveBaseClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        MoveBaseMsgsMoveBaseClient.addClientPubSub(node);
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

  // Subscribers
  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseGlobalCostmapInflaterParameterUpdates() {
    return MoveBaseGlobalCostmapInflaterParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud getMoveBaseGlobalCostmapObstaclesClearingEndpoints() {
    return MoveBaseGlobalCostmapObstaclesClearingEndpoints;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseParameterDescriptions() {
    return MoveBaseParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseGlobalCostmapStaticMapParameterUpdates() {
    return MoveBaseGlobalCostmapStaticMapParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
    return Rosout;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseLocalCostmapObstaclesParameterDescriptions() {
    return MoveBaseLocalCostmapObstaclesParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseLocalCostmapInflaterParameterUpdates() {
    return MoveBaseLocalCostmapInflaterParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.map_msgs.OccupancyGridUpdate getMoveBaseLocalCostmapCostmapUpdates() {
    return MoveBaseLocalCostmapCostmapUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped getMoveBaseCurrentGoal() {
    return MoveBaseCurrentGoal;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseLocalCostmapObstaclesParameterUpdates() {
    return MoveBaseLocalCostmapObstaclesParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.Path getMoveBaseTrajectoryPlannerROSGlobalPlan() {
    return MoveBaseTrajectoryPlannerROSGlobalPlan;
  }

  //public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped getMoveBaseGlobalCostmapFootprint() {
  //    return MoveBaseGlobalCostmapFootprint;
  //}
  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseParameterUpdates() {
    return MoveBaseParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.map_msgs.OccupancyGridUpdate getMoveBaseGlobalCostmapCostmapUpdates() {
    return MoveBaseGlobalCostmapCostmapUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseLocalCostmapParameterDescriptions() {
    return MoveBaseLocalCostmapParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseGlobalCostmapParameterUpdates() {
    return MoveBaseGlobalCostmapParameterUpdates;
  }

  //public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped getMoveBaseLocalCostmapFootprint() {
  //    return MoveBaseLocalCostmapFootprint;
  //}
  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud getMoveBaseLocalCostmapObstaclesClearingEndpoints() {
    return MoveBaseLocalCostmapObstaclesClearingEndpoints;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseTrajectoryPlannerROSParameterUpdates() {
    return MoveBaseTrajectoryPlannerROSParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist getCmdVel() {
    return CmdVel;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid getMoveBaseLocalCostmapCostmap() {
    return MoveBaseLocalCostmapCostmap;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseGlobalCostmapStaticMapParameterDescriptions() {
    return MoveBaseGlobalCostmapStaticMapParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseGlobalCostmapInflaterParameterDescriptions() {
    return MoveBaseGlobalCostmapInflaterParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.Path getMoveBaseTrajectoryPlannerROSLocalPlan() {
    return MoveBaseTrajectoryPlannerROSLocalPlan;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseTrajectoryPlannerROSParameterDescriptions() {
    return MoveBaseTrajectoryPlannerROSParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseLocalCostmapParameterUpdates() {
    return MoveBaseLocalCostmapParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveBaseGlobalCostmapObstaclesParameterUpdates() {
    return MoveBaseGlobalCostmapObstaclesParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseLocalCostmapInflaterParameterDescriptions() {
    return MoveBaseLocalCostmapInflaterParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseGlobalCostmapObstaclesParameterDescriptions() {
    return MoveBaseGlobalCostmapObstaclesParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveBaseGlobalCostmapParameterDescriptions() {
    return MoveBaseGlobalCostmapParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud2 getMoveBaseTrajectoryPlannerROSCostCloud() {
    return MoveBaseTrajectoryPlannerROSCostCloud;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.Path getMoveBaseNavfnROSPlan() {
    return MoveBaseNavfnROSPlan;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid getMoveBaseGlobalCostmapCostmap() {
    return MoveBaseGlobalCostmapCostmap;
  }

  // Publishers
  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  public void sendOdom(edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry msg) {
    OdomPublisher.publish(edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry.toRos(msg, node));
  }

  public void sendBaseScan(edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan msg) {
    BaseScanPublisher.publish(edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan.toRos(msg, node));
  }

  //public void sendMoveBaseGlobalCostmapFootprint(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped msg) {
  //    MoveBaseGlobalCostmapFootprintPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped.toRos(msg, node));
  //}
  public void sendMap(edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid msg) {
    MapPublisher.publish(edu.tufts.hrilab.diarcros.msg.nav_msgs.OccupancyGrid.toRos(msg, node));
  }

  public void sendTfStatic(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
    TfStaticPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
  }

  public void sendTf(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
    TfPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
  }

  public void sendMoveBaseSimpleGoal(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped msg) {
    MoveBaseSimpleGoalPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped.toRos(msg, node));
  }

  //public void sendMoveBaseLocalCostmapFootprint(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped msg) {
  //    MoveBaseLocalCostmapFootprintPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PolygonStamped.toRos(msg, node));
  //}
  public void sendHeadCameraDepthDownsampleCameraInfo(edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo msg) {
    HeadCameraDepthDownsampleCameraInfoPublisher.publish(edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toRos(msg, node));
  }

  public void sendHeadCameraDepthDownsampleImageRaw(edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image msg) {
    HeadCameraDepthDownsampleImageRawPublisher.publish(edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toRos(msg, node));
  }

  // Services
  public boolean callMoveBaseSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseSetParametersCondition = false;
    MoveBaseSetParametersSuccess = false;
    MoveBaseSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseSetParametersResponse = mt;
                synchronized (MoveBaseSetParametersLock) {
                  MoveBaseSetParametersCondition = true;
                  MoveBaseSetParametersSuccess = true;
                  MoveBaseSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseSetParametersLock) {
                  MoveBaseSetParametersCondition = true;
                  MoveBaseSetParametersSuccess = false;
                  MoveBaseSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseSetParametersLock) {
      while (!MoveBaseSetParametersCondition) {
        try {
          MoveBaseSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseSetParametersResponse, response);
    }
    return MoveBaseSetParametersSuccess;
  }

  public boolean callMoveBaseTrajectoryPlannerROSSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseTrajectoryPlannerROSSetParametersCondition = false;
    MoveBaseTrajectoryPlannerROSSetParametersSuccess = false;
    MoveBaseTrajectoryPlannerROSSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseTrajectoryPlannerROSSetParametersResponse = mt;
                synchronized (MoveBaseTrajectoryPlannerROSSetParametersLock) {
                  MoveBaseTrajectoryPlannerROSSetParametersCondition = true;
                  MoveBaseTrajectoryPlannerROSSetParametersSuccess = true;
                  MoveBaseTrajectoryPlannerROSSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseTrajectoryPlannerROSSetParametersLock) {
                  MoveBaseTrajectoryPlannerROSSetParametersCondition = true;
                  MoveBaseTrajectoryPlannerROSSetParametersSuccess = false;
                  MoveBaseTrajectoryPlannerROSSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseTrajectoryPlannerROSSetParametersLock) {
      while (!MoveBaseTrajectoryPlannerROSSetParametersCondition) {
        try {
          MoveBaseTrajectoryPlannerROSSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseTrajectoryPlannerROSSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseTrajectoryPlannerROSSetParametersResponse, response);
    }
    return MoveBaseTrajectoryPlannerROSSetParametersSuccess;
  }

  public boolean callMoveBaseLocalCostmapSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseLocalCostmapSetParametersCondition = false;
    MoveBaseLocalCostmapSetParametersSuccess = false;
    MoveBaseLocalCostmapSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseLocalCostmapSetParametersResponse = mt;
                synchronized (MoveBaseLocalCostmapSetParametersLock) {
                  MoveBaseLocalCostmapSetParametersCondition = true;
                  MoveBaseLocalCostmapSetParametersSuccess = true;
                  MoveBaseLocalCostmapSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseLocalCostmapSetParametersLock) {
                  MoveBaseLocalCostmapSetParametersCondition = true;
                  MoveBaseLocalCostmapSetParametersSuccess = false;
                  MoveBaseLocalCostmapSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseLocalCostmapSetParametersLock) {
      while (!MoveBaseLocalCostmapSetParametersCondition) {
        try {
          MoveBaseLocalCostmapSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseLocalCostmapSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseLocalCostmapSetParametersResponse, response);
    }
    return MoveBaseLocalCostmapSetParametersSuccess;
  }

  public boolean callMoveBaseLocalCostmapObstaclesSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseLocalCostmapObstaclesSetParametersCondition = false;
    MoveBaseLocalCostmapObstaclesSetParametersSuccess = false;
    MoveBaseLocalCostmapObstaclesSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseLocalCostmapObstaclesSetParametersResponse = mt;
                synchronized (MoveBaseLocalCostmapObstaclesSetParametersLock) {
                  MoveBaseLocalCostmapObstaclesSetParametersCondition = true;
                  MoveBaseLocalCostmapObstaclesSetParametersSuccess = true;
                  MoveBaseLocalCostmapObstaclesSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseLocalCostmapObstaclesSetParametersLock) {
                  MoveBaseLocalCostmapObstaclesSetParametersCondition = true;
                  MoveBaseLocalCostmapObstaclesSetParametersSuccess = false;
                  MoveBaseLocalCostmapObstaclesSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseLocalCostmapObstaclesSetParametersLock) {
      while (!MoveBaseLocalCostmapObstaclesSetParametersCondition) {
        try {
          MoveBaseLocalCostmapObstaclesSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseLocalCostmapObstaclesSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseLocalCostmapObstaclesSetParametersResponse, response);
    }
    return MoveBaseLocalCostmapObstaclesSetParametersSuccess;
  }

  public boolean callMoveBaseLocalCostmapInflaterSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseLocalCostmapInflaterSetParametersCondition = false;
    MoveBaseLocalCostmapInflaterSetParametersSuccess = false;
    MoveBaseLocalCostmapInflaterSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseLocalCostmapInflaterSetParametersResponse = mt;
                synchronized (MoveBaseLocalCostmapInflaterSetParametersLock) {
                  MoveBaseLocalCostmapInflaterSetParametersCondition = true;
                  MoveBaseLocalCostmapInflaterSetParametersSuccess = true;
                  MoveBaseLocalCostmapInflaterSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseLocalCostmapInflaterSetParametersLock) {
                  MoveBaseLocalCostmapInflaterSetParametersCondition = true;
                  MoveBaseLocalCostmapInflaterSetParametersSuccess = false;
                  MoveBaseLocalCostmapInflaterSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseLocalCostmapInflaterSetParametersLock) {
      while (!MoveBaseLocalCostmapInflaterSetParametersCondition) {
        try {
          MoveBaseLocalCostmapInflaterSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseLocalCostmapInflaterSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseLocalCostmapInflaterSetParametersResponse, response);
    }
    return MoveBaseLocalCostmapInflaterSetParametersSuccess;
  }

  public boolean callMoveBaseGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
    MoveBaseGetLoggersCondition = false;
    MoveBaseGetLoggersSuccess = false;
    MoveBaseGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.GetLoggersResponse>() {

              @Override
              public void onSuccess(roscpp.GetLoggersResponse mt) {
                MoveBaseGetLoggersResponse = mt;
                synchronized (MoveBaseGetLoggersLock) {
                  MoveBaseGetLoggersCondition = true;
                  MoveBaseGetLoggersSuccess = true;
                  MoveBaseGetLoggersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseGetLoggersLock) {
                  MoveBaseGetLoggersCondition = true;
                  MoveBaseGetLoggersSuccess = false;
                  MoveBaseGetLoggersLock.notify();
                }
              }
            });

    synchronized (MoveBaseGetLoggersLock) {
      while (!MoveBaseGetLoggersCondition) {
        try {
          MoveBaseGetLoggersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseGetLoggersSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(MoveBaseGetLoggersResponse, response);
    }
    return MoveBaseGetLoggersSuccess;
  }

  public boolean callMoveBaseGlobalCostmapStaticMapSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseGlobalCostmapStaticMapSetParametersCondition = false;
    MoveBaseGlobalCostmapStaticMapSetParametersSuccess = false;
    MoveBaseGlobalCostmapStaticMapSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseGlobalCostmapStaticMapSetParametersResponse = mt;
                synchronized (MoveBaseGlobalCostmapStaticMapSetParametersLock) {
                  MoveBaseGlobalCostmapStaticMapSetParametersCondition = true;
                  MoveBaseGlobalCostmapStaticMapSetParametersSuccess = true;
                  MoveBaseGlobalCostmapStaticMapSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseGlobalCostmapStaticMapSetParametersLock) {
                  MoveBaseGlobalCostmapStaticMapSetParametersCondition = true;
                  MoveBaseGlobalCostmapStaticMapSetParametersSuccess = false;
                  MoveBaseGlobalCostmapStaticMapSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseGlobalCostmapStaticMapSetParametersLock) {
      while (!MoveBaseGlobalCostmapStaticMapSetParametersCondition) {
        try {
          MoveBaseGlobalCostmapStaticMapSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseGlobalCostmapStaticMapSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseGlobalCostmapStaticMapSetParametersResponse, response);
    }
    return MoveBaseGlobalCostmapStaticMapSetParametersSuccess;
  }

  public boolean callMoveBaseGlobalCostmapInflaterSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseGlobalCostmapInflaterSetParametersCondition = false;
    MoveBaseGlobalCostmapInflaterSetParametersSuccess = false;
    MoveBaseGlobalCostmapInflaterSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseGlobalCostmapInflaterSetParametersResponse = mt;
                synchronized (MoveBaseGlobalCostmapInflaterSetParametersLock) {
                  MoveBaseGlobalCostmapInflaterSetParametersCondition = true;
                  MoveBaseGlobalCostmapInflaterSetParametersSuccess = true;
                  MoveBaseGlobalCostmapInflaterSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseGlobalCostmapInflaterSetParametersLock) {
                  MoveBaseGlobalCostmapInflaterSetParametersCondition = true;
                  MoveBaseGlobalCostmapInflaterSetParametersSuccess = false;
                  MoveBaseGlobalCostmapInflaterSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseGlobalCostmapInflaterSetParametersLock) {
      while (!MoveBaseGlobalCostmapInflaterSetParametersCondition) {
        try {
          MoveBaseGlobalCostmapInflaterSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseGlobalCostmapInflaterSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseGlobalCostmapInflaterSetParametersResponse, response);
    }
    return MoveBaseGlobalCostmapInflaterSetParametersSuccess;
  }

  public boolean callMoveBaseMakePlan(edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanRequest request, edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanResponse response) {
    MoveBaseMakePlanCondition = false;
    MoveBaseMakePlanSuccess = false;
    MoveBaseMakePlan.call(edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanRequest.toRos(request, node),
            new ServiceResponseListener<nav_msgs.GetPlanResponse>() {

              @Override
              public void onSuccess(nav_msgs.GetPlanResponse mt) {
                MoveBaseMakePlanResponse = mt;
                synchronized (MoveBaseMakePlanLock) {
                  MoveBaseMakePlanCondition = true;
                  MoveBaseMakePlanSuccess = true;
                  MoveBaseMakePlanLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseMakePlanLock) {
                  MoveBaseMakePlanCondition = true;
                  MoveBaseMakePlanSuccess = false;
                  MoveBaseMakePlanLock.notify();
                }
              }
            });

    synchronized (MoveBaseMakePlanLock) {
      while (!MoveBaseMakePlanCondition) {
        try {
          MoveBaseMakePlanLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseMakePlanSuccess) {
      edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanResponse.toAde(MoveBaseMakePlanResponse, response);
    }
    return MoveBaseMakePlanSuccess;
  }

  public boolean callMoveBaseGlobalCostmapObstaclesSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseGlobalCostmapObstaclesSetParametersCondition = false;
    MoveBaseGlobalCostmapObstaclesSetParametersSuccess = false;
    MoveBaseGlobalCostmapObstaclesSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseGlobalCostmapObstaclesSetParametersResponse = mt;
                synchronized (MoveBaseGlobalCostmapObstaclesSetParametersLock) {
                  MoveBaseGlobalCostmapObstaclesSetParametersCondition = true;
                  MoveBaseGlobalCostmapObstaclesSetParametersSuccess = true;
                  MoveBaseGlobalCostmapObstaclesSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseGlobalCostmapObstaclesSetParametersLock) {
                  MoveBaseGlobalCostmapObstaclesSetParametersCondition = true;
                  MoveBaseGlobalCostmapObstaclesSetParametersSuccess = false;
                  MoveBaseGlobalCostmapObstaclesSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseGlobalCostmapObstaclesSetParametersLock) {
      while (!MoveBaseGlobalCostmapObstaclesSetParametersCondition) {
        try {
          MoveBaseGlobalCostmapObstaclesSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseGlobalCostmapObstaclesSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseGlobalCostmapObstaclesSetParametersResponse, response);
    }
    return MoveBaseGlobalCostmapObstaclesSetParametersSuccess;
  }

  public boolean callMoveBaseNavfnROSMakePlan(edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanRequest request, edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanResponse response) {
    MoveBaseNavfnROSMakePlanCondition = false;
    MoveBaseNavfnROSMakePlanSuccess = false;
    MoveBaseNavfnROSMakePlan.call(edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanRequest.toRos(request, node),
            new ServiceResponseListener<nav_msgs.GetPlanResponse>() {

              @Override
              public void onSuccess(nav_msgs.GetPlanResponse mt) {
                MoveBaseNavfnROSMakePlanResponse = mt;
                synchronized (MoveBaseNavfnROSMakePlanLock) {
                  MoveBaseNavfnROSMakePlanCondition = true;
                  MoveBaseNavfnROSMakePlanSuccess = true;
                  MoveBaseNavfnROSMakePlanLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseNavfnROSMakePlanLock) {
                  MoveBaseNavfnROSMakePlanCondition = true;
                  MoveBaseNavfnROSMakePlanSuccess = false;
                  MoveBaseNavfnROSMakePlanLock.notify();
                }
              }
            });

    synchronized (MoveBaseNavfnROSMakePlanLock) {
      while (!MoveBaseNavfnROSMakePlanCondition) {
        try {
          MoveBaseNavfnROSMakePlanLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseNavfnROSMakePlanSuccess) {
      edu.tufts.hrilab.diarcros.msg.nav_msgs.GetPlanResponse.toAde(MoveBaseNavfnROSMakePlanResponse, response);
    }
    return MoveBaseNavfnROSMakePlanSuccess;
  }

  public boolean callMoveBaseGlobalCostmapSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveBaseGlobalCostmapSetParametersCondition = false;
    MoveBaseGlobalCostmapSetParametersSuccess = false;
    MoveBaseGlobalCostmapSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveBaseGlobalCostmapSetParametersResponse = mt;
                synchronized (MoveBaseGlobalCostmapSetParametersLock) {
                  MoveBaseGlobalCostmapSetParametersCondition = true;
                  MoveBaseGlobalCostmapSetParametersSuccess = true;
                  MoveBaseGlobalCostmapSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseGlobalCostmapSetParametersLock) {
                  MoveBaseGlobalCostmapSetParametersCondition = true;
                  MoveBaseGlobalCostmapSetParametersSuccess = false;
                  MoveBaseGlobalCostmapSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveBaseGlobalCostmapSetParametersLock) {
      while (!MoveBaseGlobalCostmapSetParametersCondition) {
        try {
          MoveBaseGlobalCostmapSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseGlobalCostmapSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveBaseGlobalCostmapSetParametersResponse, response);
    }
    return MoveBaseGlobalCostmapSetParametersSuccess;
  }

  public boolean callMoveBaseClearCostmaps(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
    MoveBaseClearCostmapsCondition = false;
    MoveBaseClearCostmapsSuccess = false;
    MoveBaseClearCostmaps.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node),
            new ServiceResponseListener<std_srvs.EmptyResponse>() {

              @Override
              public void onSuccess(std_srvs.EmptyResponse mt) {
                MoveBaseClearCostmapsResponse = mt;
                synchronized (MoveBaseClearCostmapsLock) {
                  MoveBaseClearCostmapsCondition = true;
                  MoveBaseClearCostmapsSuccess = true;
                  MoveBaseClearCostmapsLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseClearCostmapsLock) {
                  MoveBaseClearCostmapsCondition = true;
                  MoveBaseClearCostmapsSuccess = false;
                  MoveBaseClearCostmapsLock.notify();
                }
              }
            });

    synchronized (MoveBaseClearCostmapsLock) {
      while (!MoveBaseClearCostmapsCondition) {
        try {
          MoveBaseClearCostmapsLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseClearCostmapsSuccess) {
      edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(MoveBaseClearCostmapsResponse, response);
    }
    return MoveBaseClearCostmapsSuccess;
  }

  public boolean callMoveBaseSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
    MoveBaseSetLoggerLevelCondition = false;
    MoveBaseSetLoggerLevelSuccess = false;
    MoveBaseSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

              @Override
              public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                MoveBaseSetLoggerLevelResponse = mt;
                synchronized (MoveBaseSetLoggerLevelLock) {
                  MoveBaseSetLoggerLevelCondition = true;
                  MoveBaseSetLoggerLevelSuccess = true;
                  MoveBaseSetLoggerLevelLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveBaseSetLoggerLevelLock) {
                  MoveBaseSetLoggerLevelCondition = true;
                  MoveBaseSetLoggerLevelSuccess = false;
                  MoveBaseSetLoggerLevelLock.notify();
                }
              }
            });

    synchronized (MoveBaseSetLoggerLevelLock) {
      while (!MoveBaseSetLoggerLevelCondition) {
        try {
          MoveBaseSetLoggerLevelLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveBaseSetLoggerLevelSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(MoveBaseSetLoggerLevelResponse, response);
    }
    return MoveBaseSetLoggerLevelSuccess;
  }

  // Action(s) Methods
  public void cancelAllMoveBaseMsgsMoveBaseGoals() {
    MoveBaseMsgsMoveBaseClient.cancelAllGoals();
  }

  public void cancelMoveBaseMsgsMoveBaseGoal() throws RosException {
    MoveBaseMsgsMoveBaseClient.cancelGoal();
  }

  public void cancelMoveBaseMsgsMoveBaseGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    MoveBaseMsgsMoveBaseClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public move_base_msgs.MoveBaseResult getMoveBaseMsgsMoveBaseResult() throws RosException {
    return MoveBaseMsgsMoveBaseClient.getResult();
  }

  public SimpleClientGoalState getMoveBaseMsgsMoveBaseState() {
    return MoveBaseMsgsMoveBaseClient.getState();
  }

  public void sendMoveBaseMsgsMoveBaseGoal(edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal goal) throws RosException {
    MoveBaseMsgsMoveBaseClient.sendGoal(edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal.toRos(goal, node));
  }

  public void waitForMoveBaseMsgsMoveBaseResult() throws InterruptedException {
    MoveBaseMsgsMoveBaseClient.waitForResult();
  }

  public boolean waitForMoveBaseMsgsMoveBaseResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveBaseMsgsMoveBaseClient.waitForResult(timeout, units);
  }

  public void waitForMoveBaseMsgsMoveBaseServer() throws InterruptedException {
    MoveBaseMsgsMoveBaseClient.waitForServer();
  }

  public boolean waitForMoveBaseMsgsMoveBaseServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveBaseMsgsMoveBaseClient.waitForServer(timeout, units);
  }
}

