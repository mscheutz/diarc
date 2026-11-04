/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.moveit;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RosException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
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

public class MoveGroup implements GenericMoveGroup {
  Logger log = LoggerFactory.getLogger(MoveGroup.class);

  @Override
  public String getMoveGroupRosVersion() {
    return "noetic";
  }

  // dynamic namespace and remappings
  private String namespace = "/";
  private Map<GraphName, GraphName> remappings = new HashMap<>();

  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();
  // Subscription local data & locks
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveGroupSenseForPlanParameterUpdates;
  private final Object MoveGroupSenseForPlanParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
  private final Object RosoutLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveGroupPlanningSceneMonitorParameterDescriptions;
  private final Object MoveGroupPlanningSceneMonitorParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveGroupPlanExecutionParameterDescriptions;
  private final Object MoveGroupPlanExecutionParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.moveit_msgs.DisplayTrajectory MoveGroupDisplayPlannedPath;
  private final Object MoveGroupDisplayPlannedPathLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveGroupTrajectoryExecutionParameterDescriptions;
  private final Object MoveGroupTrajectoryExecutionParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveGroupOmplParameterUpdates;
  private final Object MoveGroupOmplParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveGroupOmplParameterDescriptions;
  private final Object MoveGroupOmplParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveGroupTrajectoryExecutionParameterUpdates;
  private final Object MoveGroupTrajectoryExecutionParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveGroupPlanExecutionParameterUpdates;
  private final Object MoveGroupPlanExecutionParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config MoveGroupPlanningSceneMonitorParameterUpdates;
  private final Object MoveGroupPlanningSceneMonitorParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription MoveGroupSenseForPlanParameterDescriptions;
  private final Object MoveGroupSenseForPlanParameterDescriptionsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.visualization_msgs.MarkerArray MoveGroupDisplayContacts;
  private final Object MoveGroupDisplayContactsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningScene MoveGroupMonitoredPlanningScene;
  private final Object MoveGroupMonitoredPlanningSceneLock = new Object();

  // Publishers
  private Publisher<tf2_msgs.TFMessage> TfPublisher;
  private Publisher<sensor_msgs.JointState> JointStatesPublisher;
  private Publisher<std_msgs.String> TrajectoryExecutionEventPublisher;
  private Publisher<tf2_msgs.TFMessage> TfStaticPublisher;
  private Publisher<moveit_msgs.PlanningSceneWorld> PlanningSceneWorldPublisher;
  private Publisher<moveit_msgs.AttachedCollisionObject> AttachedCollisionObjectPublisher;
  private Publisher<moveit_msgs.PlanningScene> PlanningScenePublisher;
  private Publisher<moveit_msgs.CollisionObject> CollisionObjectPublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  // Services
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> MoveGroupSetLoggerLevel;
  roscpp.SetLoggerLevelResponse MoveGroupSetLoggerLevelResponse;
  final Object MoveGroupSetLoggerLevelLock = new Object();
  boolean MoveGroupSetLoggerLevelCondition = false;
  boolean MoveGroupSetLoggerLevelSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveGroupPlanningSceneMonitorSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveGroupPlanningSceneMonitorSetParametersResponse;
  final Object MoveGroupPlanningSceneMonitorSetParametersLock = new Object();
  boolean MoveGroupPlanningSceneMonitorSetParametersCondition = false;
  boolean MoveGroupPlanningSceneMonitorSetParametersSuccess = false;
  ServiceClient<moveit_msgs.GetPlanningSceneRequest, moveit_msgs.GetPlanningSceneResponse> GetPlanningScene;
  moveit_msgs.GetPlanningSceneResponse GetPlanningSceneResponse;
  final Object GetPlanningSceneLock = new Object();
  boolean GetPlanningSceneCondition = false;
  boolean GetPlanningSceneSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveGroupPlanExecutionSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveGroupPlanExecutionSetParametersResponse;
  final Object MoveGroupPlanExecutionSetParametersLock = new Object();
  boolean MoveGroupPlanExecutionSetParametersCondition = false;
  boolean MoveGroupPlanExecutionSetParametersSuccess = false;
  ServiceClient<moveit_msgs.GetStateValidityRequest, moveit_msgs.GetStateValidityResponse> CheckStateValidity;
  moveit_msgs.GetStateValidityResponse CheckStateValidityResponse;
  final Object CheckStateValidityLock = new Object();
  boolean CheckStateValidityCondition = false;
  boolean CheckStateValiditySuccess = false;
  ServiceClient<moveit_msgs.QueryPlannerInterfacesRequest, moveit_msgs.QueryPlannerInterfacesResponse> QueryPlannerInterface;
  moveit_msgs.QueryPlannerInterfacesResponse QueryPlannerInterfaceResponse;
  final Object QueryPlannerInterfaceLock = new Object();
  boolean QueryPlannerInterfaceCondition = false;
  boolean QueryPlannerInterfaceSuccess = false;
  ServiceClient<moveit_msgs.GetMotionPlanRequest, moveit_msgs.GetMotionPlanResponse> PlanKinematicPath;
  moveit_msgs.GetMotionPlanResponse PlanKinematicPathResponse;
  final Object PlanKinematicPathLock = new Object();
  boolean PlanKinematicPathCondition = false;
  boolean PlanKinematicPathSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveGroupTrajectoryExecutionSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveGroupTrajectoryExecutionSetParametersResponse;
  final Object MoveGroupTrajectoryExecutionSetParametersLock = new Object();
  boolean MoveGroupTrajectoryExecutionSetParametersCondition = false;
  boolean MoveGroupTrajectoryExecutionSetParametersSuccess = false;
  ServiceClient<moveit_msgs.SaveMapRequest, moveit_msgs.SaveMapResponse> MoveGroupSaveMap;
  moveit_msgs.SaveMapResponse MoveGroupSaveMapResponse;
  final Object MoveGroupSaveMapLock = new Object();
  boolean MoveGroupSaveMapCondition = false;
  boolean MoveGroupSaveMapSuccess = false;
  ServiceClient<moveit_msgs.ApplyPlanningSceneRequest, moveit_msgs.ApplyPlanningSceneResponse> ApplyPlanningScene;
  moveit_msgs.ApplyPlanningSceneResponse ApplyPlanningSceneResponse;
  final Object ApplyPlanningSceneLock = new Object();
  boolean ApplyPlanningSceneCondition = false;
  boolean ApplyPlanningSceneSuccess = false;
  ServiceClient<moveit_msgs.GetPlannerParamsRequest, moveit_msgs.GetPlannerParamsResponse> GetPlannerParams;
  moveit_msgs.GetPlannerParamsResponse GetPlannerParamsResponse;
  final Object GetPlannerParamsLock = new Object();
  boolean GetPlannerParamsCondition = false;
  boolean GetPlannerParamsSuccess = false;
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> MoveGroupGetLoggers;
  roscpp.GetLoggersResponse MoveGroupGetLoggersResponse;
  final Object MoveGroupGetLoggersLock = new Object();
  boolean MoveGroupGetLoggersCondition = false;
  boolean MoveGroupGetLoggersSuccess = false;
  ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> ClearOctomap;
  std_srvs.EmptyResponse ClearOctomapResponse;
  final Object ClearOctomapLock = new Object();
  boolean ClearOctomapCondition = false;
  boolean ClearOctomapSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveGroupOmplSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveGroupOmplSetParametersResponse;
  final Object MoveGroupOmplSetParametersLock = new Object();
  boolean MoveGroupOmplSetParametersCondition = false;
  boolean MoveGroupOmplSetParametersSuccess = false;
  ServiceClient<moveit_msgs.LoadMapRequest, moveit_msgs.LoadMapResponse> MoveGroupLoadMap;
  moveit_msgs.LoadMapResponse MoveGroupLoadMapResponse;
  final Object MoveGroupLoadMapLock = new Object();
  boolean MoveGroupLoadMapCondition = false;
  boolean MoveGroupLoadMapSuccess = false;
  ServiceClient<moveit_msgs.GetCartesianPathRequest, moveit_msgs.GetCartesianPathResponse> ComputeCartesianPath;
  moveit_msgs.GetCartesianPathResponse ComputeCartesianPathResponse;
  final Object ComputeCartesianPathLock = new Object();
  boolean ComputeCartesianPathCondition = false;
  boolean ComputeCartesianPathSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> MoveGroupSenseForPlanSetParameters;
  dynamic_reconfigure.ReconfigureResponse MoveGroupSenseForPlanSetParametersResponse;
  final Object MoveGroupSenseForPlanSetParametersLock = new Object();
  boolean MoveGroupSenseForPlanSetParametersCondition = false;
  boolean MoveGroupSenseForPlanSetParametersSuccess = false;
  ServiceClient<moveit_msgs.GetPositionIKRequest, moveit_msgs.GetPositionIKResponse> ComputeIk;
  moveit_msgs.GetPositionIKResponse ComputeIkResponse;
  final Object ComputeIkLock = new Object();
  boolean ComputeIkCondition = false;
  boolean ComputeIkSuccess = false;
  ServiceClient<moveit_msgs.GetPositionFKRequest, moveit_msgs.GetPositionFKResponse> ComputeFk;
  moveit_msgs.GetPositionFKResponse ComputeFkResponse;
  final Object ComputeFkLock = new Object();
  boolean ComputeFkCondition = false;
  boolean ComputeFkSuccess = false;
  ServiceClient<moveit_msgs.SetPlannerParamsRequest, moveit_msgs.SetPlannerParamsResponse> SetPlannerParams;
  moveit_msgs.SetPlannerParamsResponse SetPlannerParamsResponse;
  final Object SetPlannerParamsLock = new Object();
  boolean SetPlannerParamsCondition = false;
  boolean SetPlannerParamsSuccess = false;
  // Action clients
  private SimpleActionClient<
          moveit_msgs.PickupActionFeedback,
          moveit_msgs.PickupActionGoal,
          moveit_msgs.PickupActionResult,
          moveit_msgs.PickupFeedback,
          moveit_msgs.PickupGoal,
          moveit_msgs.PickupResult> MoveitMsgsPickupClient;
  private SimpleActionClient<
          moveit_msgs.ExecuteTrajectoryActionFeedback,
          moveit_msgs.ExecuteTrajectoryActionGoal,
          moveit_msgs.ExecuteTrajectoryActionResult,
          moveit_msgs.ExecuteTrajectoryFeedback,
          moveit_msgs.ExecuteTrajectoryGoal,
          moveit_msgs.ExecuteTrajectoryResult> MoveitMsgsExecuteTrajectoryClient;
  private SimpleActionClient<
          moveit_msgs.MoveGroupActionFeedback,
          moveit_msgs.MoveGroupActionGoal,
          moveit_msgs.MoveGroupActionResult,
          moveit_msgs.MoveGroupFeedback,
          moveit_msgs.MoveGroupGoal,
          moveit_msgs.MoveGroupResult> MoveitMsgsMoveGroupClient;
  private SimpleActionClient<
          moveit_msgs.PlaceActionFeedback,
          moveit_msgs.PlaceActionGoal,
          moveit_msgs.PlaceActionResult,
          moveit_msgs.PlaceFeedback,
          moveit_msgs.PlaceGoal,
          moveit_msgs.PlaceResult> MoveitMsgsPlaceClient;
  private SimpleActionClient<
          control_msgs.FollowJointTrajectoryActionFeedback,
          control_msgs.FollowJointTrajectoryActionGoal,
          control_msgs.FollowJointTrajectoryActionResult,
          control_msgs.FollowJointTrajectoryFeedback,
          control_msgs.FollowJointTrajectoryGoal,
          control_msgs.FollowJointTrajectoryResult> ControlMsgsFollowJointTrajectoryClient;
  private SimpleActionClient<
          control_msgs.GripperCommandActionFeedback,
          control_msgs.GripperCommandActionGoal,
          control_msgs.GripperCommandActionResult,
          control_msgs.GripperCommandFeedback,
          control_msgs.GripperCommandGoal,
          control_msgs.GripperCommandResult> ControlMsgsGripperCommandClient;


  public MoveGroup(String namespace, Map<String, String> remappings) {
    this.namespace = namespace;
    remappings.forEach((k, v) -> this.remappings.put(GraphName.of(k), GraphName.of(v)));
    initialize();
  }

  public MoveGroup() {
    initialize();
  }

  private void initialize() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/move_group");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<dynamic_reconfigure.Config> MoveGroupSenseForPlanParameterUpdatesSub = node.newSubscriber("move_group/sense_for_plan/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveGroupSenseForPlanParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveGroupSenseForPlanParameterUpdatesLock) {
              MoveGroupSenseForPlanParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<rosgraph_msgs.Log> RosoutSub = node.newSubscriber("rosout", rosgraph_msgs.Log._TYPE);
        RosoutSub.addMessageListener(new MessageListener<rosgraph_msgs.Log>() {
          @Override
          public void onNewMessage(rosgraph_msgs.Log msg) {
            synchronized (RosoutLock) {
              Rosout = edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveGroupPlanningSceneMonitorParameterDescriptionsSub = node.newSubscriber("move_group/planning_scene_monitor/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveGroupPlanningSceneMonitorParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveGroupPlanningSceneMonitorParameterDescriptionsLock) {
              MoveGroupPlanningSceneMonitorParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveGroupPlanExecutionParameterDescriptionsSub = node.newSubscriber("move_group/plan_execution/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveGroupPlanExecutionParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveGroupPlanExecutionParameterDescriptionsLock) {
              MoveGroupPlanExecutionParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<moveit_msgs.DisplayTrajectory> MoveGroupDisplayPlannedPathSub = node.newSubscriber("move_group/display_planned_path", moveit_msgs.DisplayTrajectory._TYPE);
        MoveGroupDisplayPlannedPathSub.addMessageListener(new MessageListener<moveit_msgs.DisplayTrajectory>() {
          @Override
          public void onNewMessage(moveit_msgs.DisplayTrajectory msg) {
            synchronized (MoveGroupDisplayPlannedPathLock) {
              MoveGroupDisplayPlannedPath = edu.tufts.hrilab.diarcros.msg.moveit_msgs.DisplayTrajectory.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveGroupTrajectoryExecutionParameterDescriptionsSub = node.newSubscriber("move_group/trajectory_execution/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveGroupTrajectoryExecutionParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveGroupTrajectoryExecutionParameterDescriptionsLock) {
              MoveGroupTrajectoryExecutionParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveGroupOmplParameterUpdatesSub = node.newSubscriber("move_group/ompl/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveGroupOmplParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveGroupOmplParameterUpdatesLock) {
              MoveGroupOmplParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveGroupOmplParameterDescriptionsSub = node.newSubscriber("move_group/ompl/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveGroupOmplParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveGroupOmplParameterDescriptionsLock) {
              MoveGroupOmplParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveGroupTrajectoryExecutionParameterUpdatesSub = node.newSubscriber("move_group/trajectory_execution/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveGroupTrajectoryExecutionParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveGroupTrajectoryExecutionParameterUpdatesLock) {
              MoveGroupTrajectoryExecutionParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveGroupPlanExecutionParameterUpdatesSub = node.newSubscriber("move_group/plan_execution/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveGroupPlanExecutionParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveGroupPlanExecutionParameterUpdatesLock) {
              MoveGroupPlanExecutionParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.Config> MoveGroupPlanningSceneMonitorParameterUpdatesSub = node.newSubscriber("move_group/planning_scene_monitor/parameter_updates", dynamic_reconfigure.Config._TYPE);
        MoveGroupPlanningSceneMonitorParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (MoveGroupPlanningSceneMonitorParameterUpdatesLock) {
              MoveGroupPlanningSceneMonitorParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> MoveGroupSenseForPlanParameterDescriptionsSub = node.newSubscriber("move_group/sense_for_plan/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        MoveGroupSenseForPlanParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (MoveGroupSenseForPlanParameterDescriptionsLock) {
              MoveGroupSenseForPlanParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        Subscriber<visualization_msgs.MarkerArray> MoveGroupDisplayContactsSub = node.newSubscriber("move_group/display_contacts", visualization_msgs.MarkerArray._TYPE);
        MoveGroupDisplayContactsSub.addMessageListener(new MessageListener<visualization_msgs.MarkerArray>() {
          @Override
          public void onNewMessage(visualization_msgs.MarkerArray msg) {
            synchronized (MoveGroupDisplayContactsLock) {
              MoveGroupDisplayContacts = edu.tufts.hrilab.diarcros.msg.visualization_msgs.MarkerArray.toAde(msg);
            }
          }
        });
        Subscriber<moveit_msgs.PlanningScene> MoveGroupMonitoredPlanningSceneSub = node.newSubscriber("move_group/monitored_planning_scene", moveit_msgs.PlanningScene._TYPE);
        MoveGroupMonitoredPlanningSceneSub.addMessageListener(new MessageListener<moveit_msgs.PlanningScene>() {
          @Override
          public void onNewMessage(moveit_msgs.PlanningScene msg) {
            synchronized (MoveGroupMonitoredPlanningSceneLock) {
              MoveGroupMonitoredPlanningScene = edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningScene.toAde(msg);
            }
          }
        });
        // Publishers
        TfPublisher = node.newPublisher("tf", tf2_msgs.TFMessage._TYPE);
        JointStatesPublisher = node.newPublisher("joint_states", sensor_msgs.JointState._TYPE);
        TrajectoryExecutionEventPublisher = node.newPublisher("trajectory_execution_event", std_msgs.String._TYPE);
        TfStaticPublisher = node.newPublisher("tf_static", tf2_msgs.TFMessage._TYPE);
        PlanningSceneWorldPublisher = node.newPublisher("planning_scene_world", moveit_msgs.PlanningSceneWorld._TYPE);
        AttachedCollisionObjectPublisher = node.newPublisher("attached_collision_object", moveit_msgs.AttachedCollisionObject._TYPE);
        PlanningScenePublisher = node.newPublisher("planning_scene", moveit_msgs.PlanningScene._TYPE);
        CollisionObjectPublisher = node.newPublisher("collision_object", moveit_msgs.CollisionObject._TYPE);
        ClockPublisher = node.newPublisher("clock", rosgraph_msgs.Clock._TYPE);
        //Services
        try {
          MoveGroupSetLoggerLevel = node.newServiceClient("move_group/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupPlanningSceneMonitorSetParameters = node.newServiceClient("move_group/planning_scene_monitor/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          GetPlanningScene = node.newServiceClient("get_planning_scene", moveit_msgs.GetPlanningScene._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupPlanExecutionSetParameters = node.newServiceClient("move_group/plan_execution/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          CheckStateValidity = node.newServiceClient("check_state_validity", moveit_msgs.GetStateValidity._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          QueryPlannerInterface = node.newServiceClient("query_planner_interface", moveit_msgs.QueryPlannerInterfaces._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          PlanKinematicPath = node.newServiceClient("plan_kinematic_path", moveit_msgs.GetMotionPlan._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupTrajectoryExecutionSetParameters = node.newServiceClient("move_group/trajectory_execution/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupSaveMap = node.newServiceClient("move_group/save_map", moveit_msgs.SaveMap._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ApplyPlanningScene = node.newServiceClient("apply_planning_scene", moveit_msgs.ApplyPlanningScene._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          GetPlannerParams = node.newServiceClient("get_planner_params", moveit_msgs.GetPlannerParams._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupGetLoggers = node.newServiceClient("move_group/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ClearOctomap = node.newServiceClient("clear_octomap", std_srvs.Empty._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupOmplSetParameters = node.newServiceClient("move_group/ompl/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupLoadMap = node.newServiceClient("move_group/load_map", moveit_msgs.LoadMap._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ComputeCartesianPath = node.newServiceClient("compute_cartesian_path", moveit_msgs.GetCartesianPath._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          MoveGroupSenseForPlanSetParameters = node.newServiceClient("move_group/sense_for_plan/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ComputeIk = node.newServiceClient("compute_ik", moveit_msgs.GetPositionIK._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ComputeFk = node.newServiceClient("compute_fk", moveit_msgs.GetPositionFK._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          SetPlannerParams = node.newServiceClient("set_planner_params", moveit_msgs.SetPlannerParams._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        // Action Client
        try {
          MoveitMsgsPickupClient = new SimpleActionClient<>("pickup",
                  new ActionSpec(moveit_msgs.PickupAction.class,
                          "moveit_msgs/PickupAction",
                          "moveit_msgs/PickupActionFeedback",
                          "moveit_msgs/PickupActionGoal",
                          "moveit_msgs/PickupActionResult",
                          "moveit_msgs/PickupFeedback",
                          "moveit_msgs/PickupGoal",
                          "moveit_msgs/PickupResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (MoveitMsgsPickupClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        MoveitMsgsPickupClient.addClientPubSub(node);
        try {
          MoveitMsgsExecuteTrajectoryClient = new SimpleActionClient<>("execute_trajectory",
                  new ActionSpec(moveit_msgs.ExecuteTrajectoryAction.class,
                          "moveit_msgs/ExecuteTrajectoryAction",
                          "moveit_msgs/ExecuteTrajectoryActionFeedback",
                          "moveit_msgs/ExecuteTrajectoryActionGoal",
                          "moveit_msgs/ExecuteTrajectoryActionResult",
                          "moveit_msgs/ExecuteTrajectoryFeedback",
                          "moveit_msgs/ExecuteTrajectoryGoal",
                          "moveit_msgs/ExecuteTrajectoryResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (MoveitMsgsExecuteTrajectoryClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        MoveitMsgsExecuteTrajectoryClient.addClientPubSub(node);
        try {
          MoveitMsgsMoveGroupClient = new SimpleActionClient<>("move_group",
                  new ActionSpec(moveit_msgs.MoveGroupAction.class,
                          "moveit_msgs/MoveGroupAction",
                          "moveit_msgs/MoveGroupActionFeedback",
                          "moveit_msgs/MoveGroupActionGoal",
                          "moveit_msgs/MoveGroupActionResult",
                          "moveit_msgs/MoveGroupFeedback",
                          "moveit_msgs/MoveGroupGoal",
                          "moveit_msgs/MoveGroupResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (MoveitMsgsMoveGroupClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        MoveitMsgsMoveGroupClient.addClientPubSub(node);
        try {
          MoveitMsgsPlaceClient = new SimpleActionClient<>("place",
                  new ActionSpec(moveit_msgs.PlaceAction.class,
                          "moveit_msgs/PlaceAction",
                          "moveit_msgs/PlaceActionFeedback",
                          "moveit_msgs/PlaceActionGoal",
                          "moveit_msgs/PlaceActionResult",
                          "moveit_msgs/PlaceFeedback",
                          "moveit_msgs/PlaceGoal",
                          "moveit_msgs/PlaceResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (MoveitMsgsPlaceClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        MoveitMsgsPlaceClient.addClientPubSub(node);
        try {
          ControlMsgsFollowJointTrajectoryClient = new SimpleActionClient<>("move_group",
                  new ActionSpec(control_msgs.FollowJointTrajectoryAction.class,
                          "control_msgs/FollowJointTrajectoryAction",
                          "control_msgs/FollowJointTrajectoryActionFeedback",
                          "control_msgs/FollowJointTrajectoryActionGoal",
                          "control_msgs/FollowJointTrajectoryActionResult",
                          "control_msgs/FollowJointTrajectoryFeedback",
                          "control_msgs/FollowJointTrajectoryGoal",
                          "control_msgs/FollowJointTrajectoryResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (ControlMsgsFollowJointTrajectoryClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        ControlMsgsFollowJointTrajectoryClient.addClientPubSub(node);
        try {
          ControlMsgsGripperCommandClient = new SimpleActionClient<>("move_group",
                  new ActionSpec(control_msgs.GripperCommandAction.class,
                          "control_msgs/GripperCommandAction",
                          "control_msgs/GripperCommandActionFeedback",
                          "control_msgs/GripperCommandActionGoal",
                          "control_msgs/GripperCommandActionResult",
                          "control_msgs/GripperCommandFeedback",
                          "control_msgs/GripperCommandGoal",
                          "control_msgs/GripperCommandResult"));


        } catch (RosException e) {
          e.printStackTrace();
        }
        while (ControlMsgsGripperCommandClient == null)
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        ControlMsgsGripperCommandClient.addClientPubSub(node);
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
      nodeConfiguration.setParentResolver(NameResolver.newFromNamespaceAndRemappings(namespace, remappings));
      nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(nodeMain, nodeConfiguration);
    } catch (URISyntaxException e) {
      System.err.println("Error trying to create URI: " + e);
    }
  }

  @Override
  public void shutdown() {
    node.shutdown();
  }

  @Override
  public boolean isConnected() {
    return false; //TODO
  }

  @Override
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
  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupSenseForPlanParameterUpdates() {
    return MoveGroupSenseForPlanParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupPlanningSceneMonitorParameterDescriptions() {
    return MoveGroupPlanningSceneMonitorParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupPlanExecutionParameterDescriptions() {
    return MoveGroupPlanExecutionParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.moveit_msgs.DisplayTrajectory getMoveGroupDisplayPlannedPath() {
    return MoveGroupDisplayPlannedPath;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupTrajectoryExecutionParameterDescriptions() {
    return MoveGroupTrajectoryExecutionParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupOmplParameterUpdates() {
    return MoveGroupOmplParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupOmplParameterDescriptions() {
    return MoveGroupOmplParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupTrajectoryExecutionParameterUpdates() {
    return MoveGroupTrajectoryExecutionParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupPlanExecutionParameterUpdates() {
    return MoveGroupPlanExecutionParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupPlanningSceneMonitorParameterUpdates() {
    return MoveGroupPlanningSceneMonitorParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupSenseForPlanParameterDescriptions() {
    return MoveGroupSenseForPlanParameterDescriptions;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.visualization_msgs.MarkerArray getMoveGroupDisplayContacts() {
    return MoveGroupDisplayContacts;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningScene getMoveGroupMonitoredPlanningScene() {
    return MoveGroupMonitoredPlanningScene;
  }

  // Publishers
  public void sendTf(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
    TfPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
  }

  public void sendJointStates(edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState msg) {
    JointStatesPublisher.publish(edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toRos(msg, node));
  }

  public void sendTrajectoryExecutionEvent(edu.tufts.hrilab.diarcros.msg.std_msgs.String msg) {
    TrajectoryExecutionEventPublisher.publish(edu.tufts.hrilab.diarcros.msg.std_msgs.String.toRos(msg, node));
  }

  public void sendTfStatic(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
    TfStaticPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
  }

  public void sendPlanningSceneWorld(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningSceneWorld msg) {
    PlanningSceneWorldPublisher.publish(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningSceneWorld.toRos(msg, node));
  }

  public void sendAttachedCollisionObject(edu.tufts.hrilab.diarcros.msg.moveit_msgs.AttachedCollisionObject msg) {
    AttachedCollisionObjectPublisher.publish(edu.tufts.hrilab.diarcros.msg.moveit_msgs.AttachedCollisionObject.toRos(msg, node));
  }

  public void sendPlanningScene(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningScene msg) {
    PlanningScenePublisher.publish(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningScene.toRos(msg, node));
  }

  public void sendCollisionObject(edu.tufts.hrilab.diarcros.msg.moveit_msgs.CollisionObject msg) {
    CollisionObjectPublisher.publish(edu.tufts.hrilab.diarcros.msg.moveit_msgs.CollisionObject.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  // Services
  public boolean callMoveGroupSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
    MoveGroupSetLoggerLevelCondition = false;
    MoveGroupSetLoggerLevelSuccess = false;
    MoveGroupSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

              @Override
              public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                MoveGroupSetLoggerLevelResponse = mt;
                synchronized (MoveGroupSetLoggerLevelLock) {
                  MoveGroupSetLoggerLevelCondition = true;
                  MoveGroupSetLoggerLevelSuccess = true;
                  MoveGroupSetLoggerLevelLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupSetLoggerLevelLock) {
                  MoveGroupSetLoggerLevelCondition = true;
                  MoveGroupSetLoggerLevelSuccess = false;
                  MoveGroupSetLoggerLevelLock.notify();
                }
              }
            });

    synchronized (MoveGroupSetLoggerLevelLock) {
      while (!MoveGroupSetLoggerLevelCondition) {
        try {
          MoveGroupSetLoggerLevelLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupSetLoggerLevelSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(MoveGroupSetLoggerLevelResponse, response);
    }
    return MoveGroupSetLoggerLevelSuccess;
  }

  public boolean callMoveGroupPlanningSceneMonitorSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveGroupPlanningSceneMonitorSetParametersCondition = false;
    MoveGroupPlanningSceneMonitorSetParametersSuccess = false;
    MoveGroupPlanningSceneMonitorSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveGroupPlanningSceneMonitorSetParametersResponse = mt;
                synchronized (MoveGroupPlanningSceneMonitorSetParametersLock) {
                  MoveGroupPlanningSceneMonitorSetParametersCondition = true;
                  MoveGroupPlanningSceneMonitorSetParametersSuccess = true;
                  MoveGroupPlanningSceneMonitorSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupPlanningSceneMonitorSetParametersLock) {
                  MoveGroupPlanningSceneMonitorSetParametersCondition = true;
                  MoveGroupPlanningSceneMonitorSetParametersSuccess = false;
                  MoveGroupPlanningSceneMonitorSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveGroupPlanningSceneMonitorSetParametersLock) {
      while (!MoveGroupPlanningSceneMonitorSetParametersCondition) {
        try {
          MoveGroupPlanningSceneMonitorSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupPlanningSceneMonitorSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveGroupPlanningSceneMonitorSetParametersResponse, response);
    }
    return MoveGroupPlanningSceneMonitorSetParametersSuccess;
  }

  public boolean callGetPlanningScene(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlanningSceneRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlanningSceneResponse response) {
    GetPlanningSceneCondition = false;
    GetPlanningSceneSuccess = false;
    GetPlanningScene.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlanningSceneRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.GetPlanningSceneResponse>() {

              @Override
              public void onSuccess(moveit_msgs.GetPlanningSceneResponse mt) {
                GetPlanningSceneResponse = mt;
                synchronized (GetPlanningSceneLock) {
                  GetPlanningSceneCondition = true;
                  GetPlanningSceneSuccess = true;
                  GetPlanningSceneLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (GetPlanningSceneLock) {
                  GetPlanningSceneCondition = true;
                  GetPlanningSceneSuccess = false;
                  GetPlanningSceneLock.notify();
                }
              }
            });

    synchronized (GetPlanningSceneLock) {
      while (!GetPlanningSceneCondition) {
        try {
          GetPlanningSceneLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (GetPlanningSceneSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlanningSceneResponse.toAde(GetPlanningSceneResponse, response);
    }
    return GetPlanningSceneSuccess;
  }

  public boolean callMoveGroupPlanExecutionSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveGroupPlanExecutionSetParametersCondition = false;
    MoveGroupPlanExecutionSetParametersSuccess = false;
    MoveGroupPlanExecutionSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveGroupPlanExecutionSetParametersResponse = mt;
                synchronized (MoveGroupPlanExecutionSetParametersLock) {
                  MoveGroupPlanExecutionSetParametersCondition = true;
                  MoveGroupPlanExecutionSetParametersSuccess = true;
                  MoveGroupPlanExecutionSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupPlanExecutionSetParametersLock) {
                  MoveGroupPlanExecutionSetParametersCondition = true;
                  MoveGroupPlanExecutionSetParametersSuccess = false;
                  MoveGroupPlanExecutionSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveGroupPlanExecutionSetParametersLock) {
      while (!MoveGroupPlanExecutionSetParametersCondition) {
        try {
          MoveGroupPlanExecutionSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupPlanExecutionSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveGroupPlanExecutionSetParametersResponse, response);
    }
    return MoveGroupPlanExecutionSetParametersSuccess;
  }

  public boolean callCheckStateValidity(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetStateValidityRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetStateValidityResponse response) {
    CheckStateValidityCondition = false;
    CheckStateValiditySuccess = false;
    CheckStateValidity.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetStateValidityRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.GetStateValidityResponse>() {

              @Override
              public void onSuccess(moveit_msgs.GetStateValidityResponse mt) {
                CheckStateValidityResponse = mt;
                synchronized (CheckStateValidityLock) {
                  CheckStateValidityCondition = true;
                  CheckStateValiditySuccess = true;
                  CheckStateValidityLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (CheckStateValidityLock) {
                  CheckStateValidityCondition = true;
                  CheckStateValiditySuccess = false;
                  CheckStateValidityLock.notify();
                }
              }
            });

    synchronized (CheckStateValidityLock) {
      while (!CheckStateValidityCondition) {
        try {
          CheckStateValidityLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (CheckStateValiditySuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetStateValidityResponse.toAde(CheckStateValidityResponse, response);
    }
    return CheckStateValiditySuccess;
  }

  public boolean callQueryPlannerInterface(edu.tufts.hrilab.diarcros.msg.moveit_msgs.QueryPlannerInterfacesRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.QueryPlannerInterfacesResponse response) {
    QueryPlannerInterfaceCondition = false;
    QueryPlannerInterfaceSuccess = false;
    QueryPlannerInterface.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.QueryPlannerInterfacesRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.QueryPlannerInterfacesResponse>() {

              @Override
              public void onSuccess(moveit_msgs.QueryPlannerInterfacesResponse mt) {
                QueryPlannerInterfaceResponse = mt;
                synchronized (QueryPlannerInterfaceLock) {
                  QueryPlannerInterfaceCondition = true;
                  QueryPlannerInterfaceSuccess = true;
                  QueryPlannerInterfaceLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (QueryPlannerInterfaceLock) {
                  QueryPlannerInterfaceCondition = true;
                  QueryPlannerInterfaceSuccess = false;
                  QueryPlannerInterfaceLock.notify();
                }
              }
            });

    synchronized (QueryPlannerInterfaceLock) {
      while (!QueryPlannerInterfaceCondition) {
        try {
          QueryPlannerInterfaceLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (QueryPlannerInterfaceSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.QueryPlannerInterfacesResponse.toAde(QueryPlannerInterfaceResponse, response);
    }
    return QueryPlannerInterfaceSuccess;
  }

  public boolean callPlanKinematicPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetMotionPlanRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetMotionPlanResponse response) {
    PlanKinematicPathCondition = false;
    PlanKinematicPathSuccess = false;
    log.debug("calling ros plan kinematic");
    PlanKinematicPath.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetMotionPlanRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.GetMotionPlanResponse>() {

              @Override
              public void onSuccess(moveit_msgs.GetMotionPlanResponse mt) {
                PlanKinematicPathResponse = mt;
                synchronized (PlanKinematicPathLock) {
                  PlanKinematicPathCondition = true;
                  PlanKinematicPathSuccess = true;
                  PlanKinematicPathLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (PlanKinematicPathLock) {
                  PlanKinematicPathCondition = true;
                  PlanKinematicPathSuccess = false;
                  PlanKinematicPathLock.notify();
                }
              }
            });

    synchronized (PlanKinematicPathLock) {
      while (!PlanKinematicPathCondition) {
        try {
          PlanKinematicPathLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (PlanKinematicPathSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetMotionPlanResponse.toAde(PlanKinematicPathResponse, response);
    }
    log.debug("finished calling ros plan kinematic");
    return PlanKinematicPathSuccess;
  }

  public boolean callMoveGroupTrajectoryExecutionSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveGroupTrajectoryExecutionSetParametersCondition = false;
    MoveGroupTrajectoryExecutionSetParametersSuccess = false;
    MoveGroupTrajectoryExecutionSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveGroupTrajectoryExecutionSetParametersResponse = mt;
                synchronized (MoveGroupTrajectoryExecutionSetParametersLock) {
                  MoveGroupTrajectoryExecutionSetParametersCondition = true;
                  MoveGroupTrajectoryExecutionSetParametersSuccess = true;
                  MoveGroupTrajectoryExecutionSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupTrajectoryExecutionSetParametersLock) {
                  MoveGroupTrajectoryExecutionSetParametersCondition = true;
                  MoveGroupTrajectoryExecutionSetParametersSuccess = false;
                  MoveGroupTrajectoryExecutionSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveGroupTrajectoryExecutionSetParametersLock) {
      while (!MoveGroupTrajectoryExecutionSetParametersCondition) {
        try {
          MoveGroupTrajectoryExecutionSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupTrajectoryExecutionSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveGroupTrajectoryExecutionSetParametersResponse, response);
    }
    return MoveGroupTrajectoryExecutionSetParametersSuccess;
  }

  public boolean callMoveGroupSaveMap(edu.tufts.hrilab.diarcros.msg.moveit_msgs.SaveMapRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.SaveMapResponse response) {
    MoveGroupSaveMapCondition = false;
    MoveGroupSaveMapSuccess = false;
    MoveGroupSaveMap.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.SaveMapRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.SaveMapResponse>() {

              @Override
              public void onSuccess(moveit_msgs.SaveMapResponse mt) {
                MoveGroupSaveMapResponse = mt;
                synchronized (MoveGroupSaveMapLock) {
                  MoveGroupSaveMapCondition = true;
                  MoveGroupSaveMapSuccess = true;
                  MoveGroupSaveMapLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupSaveMapLock) {
                  MoveGroupSaveMapCondition = true;
                  MoveGroupSaveMapSuccess = false;
                  MoveGroupSaveMapLock.notify();
                }
              }
            });

    synchronized (MoveGroupSaveMapLock) {
      while (!MoveGroupSaveMapCondition) {
        try {
          MoveGroupSaveMapLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupSaveMapSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.SaveMapResponse.toAde(MoveGroupSaveMapResponse, response);
    }
    return MoveGroupSaveMapSuccess;
  }

  public boolean callApplyPlanningScene(edu.tufts.hrilab.diarcros.msg.moveit_msgs.ApplyPlanningSceneRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.ApplyPlanningSceneResponse response) {
    ApplyPlanningSceneCondition = false;
    ApplyPlanningSceneSuccess = false;
    ApplyPlanningScene.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.ApplyPlanningSceneRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.ApplyPlanningSceneResponse>() {

              @Override
              public void onSuccess(moveit_msgs.ApplyPlanningSceneResponse mt) {
                ApplyPlanningSceneResponse = mt;
                synchronized (ApplyPlanningSceneLock) {
                  ApplyPlanningSceneCondition = true;
                  ApplyPlanningSceneSuccess = true;
                  ApplyPlanningSceneLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ApplyPlanningSceneLock) {
                  ApplyPlanningSceneCondition = true;
                  ApplyPlanningSceneSuccess = false;
                  ApplyPlanningSceneLock.notify();
                }
              }
            });

    synchronized (ApplyPlanningSceneLock) {
      while (!ApplyPlanningSceneCondition) {
        try {
          ApplyPlanningSceneLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ApplyPlanningSceneSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.ApplyPlanningSceneResponse.toAde(ApplyPlanningSceneResponse, response);
    }
    return ApplyPlanningSceneSuccess;
  }

  public boolean callGetPlannerParams(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlannerParamsRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlannerParamsResponse response) {
    GetPlannerParamsCondition = false;
    GetPlannerParamsSuccess = false;
    GetPlannerParams.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlannerParamsRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.GetPlannerParamsResponse>() {

              @Override
              public void onSuccess(moveit_msgs.GetPlannerParamsResponse mt) {
                GetPlannerParamsResponse = mt;
                synchronized (GetPlannerParamsLock) {
                  GetPlannerParamsCondition = true;
                  GetPlannerParamsSuccess = true;
                  GetPlannerParamsLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (GetPlannerParamsLock) {
                  GetPlannerParamsCondition = true;
                  GetPlannerParamsSuccess = false;
                  GetPlannerParamsLock.notify();
                }
              }
            });

    synchronized (GetPlannerParamsLock) {
      while (!GetPlannerParamsCondition) {
        try {
          GetPlannerParamsLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (GetPlannerParamsSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlannerParamsResponse.toAde(GetPlannerParamsResponse, response);
    }
    return GetPlannerParamsSuccess;
  }

  public boolean callMoveGroupGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
    MoveGroupGetLoggersCondition = false;
    MoveGroupGetLoggersSuccess = false;
    MoveGroupGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.GetLoggersResponse>() {

              @Override
              public void onSuccess(roscpp.GetLoggersResponse mt) {
                MoveGroupGetLoggersResponse = mt;
                synchronized (MoveGroupGetLoggersLock) {
                  MoveGroupGetLoggersCondition = true;
                  MoveGroupGetLoggersSuccess = true;
                  MoveGroupGetLoggersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupGetLoggersLock) {
                  MoveGroupGetLoggersCondition = true;
                  MoveGroupGetLoggersSuccess = false;
                  MoveGroupGetLoggersLock.notify();
                }
              }
            });

    synchronized (MoveGroupGetLoggersLock) {
      while (!MoveGroupGetLoggersCondition) {
        try {
          MoveGroupGetLoggersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupGetLoggersSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(MoveGroupGetLoggersResponse, response);
    }
    return MoveGroupGetLoggersSuccess;
  }

  public boolean callClearOctomap(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
    ClearOctomapCondition = false;
    ClearOctomapSuccess = false;
    ClearOctomap.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node),
            new ServiceResponseListener<std_srvs.EmptyResponse>() {

              @Override
              public void onSuccess(std_srvs.EmptyResponse mt) {
                ClearOctomapResponse = mt;
                synchronized (ClearOctomapLock) {
                  ClearOctomapCondition = true;
                  ClearOctomapSuccess = true;
                  ClearOctomapLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ClearOctomapLock) {
                  ClearOctomapCondition = true;
                  ClearOctomapSuccess = false;
                  ClearOctomapLock.notify();
                }
              }
            });

    synchronized (ClearOctomapLock) {
      while (!ClearOctomapCondition) {
        try {
          ClearOctomapLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ClearOctomapSuccess) {
      edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(ClearOctomapResponse, response);
    }
    return ClearOctomapSuccess;
  }

  public boolean callMoveGroupOmplSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveGroupOmplSetParametersCondition = false;
    MoveGroupOmplSetParametersSuccess = false;
    MoveGroupOmplSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveGroupOmplSetParametersResponse = mt;
                synchronized (MoveGroupOmplSetParametersLock) {
                  MoveGroupOmplSetParametersCondition = true;
                  MoveGroupOmplSetParametersSuccess = true;
                  MoveGroupOmplSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupOmplSetParametersLock) {
                  MoveGroupOmplSetParametersCondition = true;
                  MoveGroupOmplSetParametersSuccess = false;
                  MoveGroupOmplSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveGroupOmplSetParametersLock) {
      while (!MoveGroupOmplSetParametersCondition) {
        try {
          MoveGroupOmplSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupOmplSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveGroupOmplSetParametersResponse, response);
    }
    return MoveGroupOmplSetParametersSuccess;
  }

  public boolean callMoveGroupLoadMap(edu.tufts.hrilab.diarcros.msg.moveit_msgs.LoadMapRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.LoadMapResponse response) {
    MoveGroupLoadMapCondition = false;
    MoveGroupLoadMapSuccess = false;
    MoveGroupLoadMap.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.LoadMapRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.LoadMapResponse>() {

              @Override
              public void onSuccess(moveit_msgs.LoadMapResponse mt) {
                MoveGroupLoadMapResponse = mt;
                synchronized (MoveGroupLoadMapLock) {
                  MoveGroupLoadMapCondition = true;
                  MoveGroupLoadMapSuccess = true;
                  MoveGroupLoadMapLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupLoadMapLock) {
                  MoveGroupLoadMapCondition = true;
                  MoveGroupLoadMapSuccess = false;
                  MoveGroupLoadMapLock.notify();
                }
              }
            });

    synchronized (MoveGroupLoadMapLock) {
      while (!MoveGroupLoadMapCondition) {
        try {
          MoveGroupLoadMapLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupLoadMapSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.LoadMapResponse.toAde(MoveGroupLoadMapResponse, response);
    }
    return MoveGroupLoadMapSuccess;
  }

  public boolean callComputeCartesianPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetCartesianPathRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetCartesianPathResponse response) {
    ComputeCartesianPathCondition = false;
    ComputeCartesianPathSuccess = false;
    ComputeCartesianPath.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetCartesianPathRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.GetCartesianPathResponse>() {

              @Override
              public void onSuccess(moveit_msgs.GetCartesianPathResponse mt) {
                ComputeCartesianPathResponse = mt;
                synchronized (ComputeCartesianPathLock) {
                  ComputeCartesianPathCondition = true;
                  ComputeCartesianPathSuccess = true;
                  ComputeCartesianPathLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ComputeCartesianPathLock) {
                  ComputeCartesianPathCondition = true;
                  ComputeCartesianPathSuccess = false;
                  ComputeCartesianPathLock.notify();
                }
              }
            });

    synchronized (ComputeCartesianPathLock) {
      while (!ComputeCartesianPathCondition) {
        try {
          ComputeCartesianPathLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ComputeCartesianPathSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetCartesianPathResponse.toAde(ComputeCartesianPathResponse, response);
    }
    return ComputeCartesianPathSuccess;
  }

  public boolean callMoveGroupSenseForPlanSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    MoveGroupSenseForPlanSetParametersCondition = false;
    MoveGroupSenseForPlanSetParametersSuccess = false;
    MoveGroupSenseForPlanSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                MoveGroupSenseForPlanSetParametersResponse = mt;
                synchronized (MoveGroupSenseForPlanSetParametersLock) {
                  MoveGroupSenseForPlanSetParametersCondition = true;
                  MoveGroupSenseForPlanSetParametersSuccess = true;
                  MoveGroupSenseForPlanSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (MoveGroupSenseForPlanSetParametersLock) {
                  MoveGroupSenseForPlanSetParametersCondition = true;
                  MoveGroupSenseForPlanSetParametersSuccess = false;
                  MoveGroupSenseForPlanSetParametersLock.notify();
                }
              }
            });

    synchronized (MoveGroupSenseForPlanSetParametersLock) {
      while (!MoveGroupSenseForPlanSetParametersCondition) {
        try {
          MoveGroupSenseForPlanSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (MoveGroupSenseForPlanSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(MoveGroupSenseForPlanSetParametersResponse, response);
    }
    return MoveGroupSenseForPlanSetParametersSuccess;
  }

  public boolean callComputeIk(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionIKRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionIKResponse response) {
    ComputeIkCondition = false;
    ComputeIkSuccess = false;
    ComputeIk.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionIKRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.GetPositionIKResponse>() {

              @Override
              public void onSuccess(moveit_msgs.GetPositionIKResponse mt) {
                ComputeIkResponse = mt;
                synchronized (ComputeIkLock) {
                  ComputeIkCondition = true;
                  ComputeIkSuccess = true;
                  ComputeIkLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ComputeIkLock) {
                  ComputeIkCondition = true;
                  ComputeIkSuccess = false;
                  ComputeIkLock.notify();
                }
              }
            });

    synchronized (ComputeIkLock) {
      while (!ComputeIkCondition) {
        try {
          ComputeIkLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ComputeIkSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionIKResponse.toAde(ComputeIkResponse, response);
    }
    return ComputeIkSuccess;
  }

  public boolean callComputeFk(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionFKRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionFKResponse response) {
    ComputeFkCondition = false;
    ComputeFkSuccess = false;
    ComputeFk.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionFKRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.GetPositionFKResponse>() {

              @Override
              public void onSuccess(moveit_msgs.GetPositionFKResponse mt) {
                ComputeFkResponse = mt;
                synchronized (ComputeFkLock) {
                  ComputeFkCondition = true;
                  ComputeFkSuccess = true;
                  ComputeFkLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ComputeFkLock) {
                  ComputeFkCondition = true;
                  ComputeFkSuccess = false;
                  ComputeFkLock.notify();
                }
              }
            });

    synchronized (ComputeFkLock) {
      while (!ComputeFkCondition) {
        try {
          ComputeFkLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ComputeFkSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionFKResponse.toAde(ComputeFkResponse, response);
    }
    return ComputeFkSuccess;
  }

  public boolean callSetPlannerParams(edu.tufts.hrilab.diarcros.msg.moveit_msgs.SetPlannerParamsRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.SetPlannerParamsResponse response) {
    SetPlannerParamsCondition = false;
    SetPlannerParamsSuccess = false;
    SetPlannerParams.call(edu.tufts.hrilab.diarcros.msg.moveit_msgs.SetPlannerParamsRequest.toRos(request, node),
            new ServiceResponseListener<moveit_msgs.SetPlannerParamsResponse>() {

              @Override
              public void onSuccess(moveit_msgs.SetPlannerParamsResponse mt) {
                SetPlannerParamsResponse = mt;
                synchronized (SetPlannerParamsLock) {
                  SetPlannerParamsCondition = true;
                  SetPlannerParamsSuccess = true;
                  SetPlannerParamsLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SetPlannerParamsLock) {
                  SetPlannerParamsCondition = true;
                  SetPlannerParamsSuccess = false;
                  SetPlannerParamsLock.notify();
                }
              }
            });

    synchronized (SetPlannerParamsLock) {
      while (!SetPlannerParamsCondition) {
        try {
          SetPlannerParamsLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (SetPlannerParamsSuccess) {
      edu.tufts.hrilab.diarcros.msg.moveit_msgs.SetPlannerParamsResponse.toAde(SetPlannerParamsResponse, response);
    }
    return SetPlannerParamsSuccess;
  }

  // Action(s) Methods
//    public void cancelAllMoveitMsgsPickupGoals() {
//        MoveitMsgsPickupClient.cancelAllGoals();
//    }
//
//    public void cancelMoveitMsgsPickupGoal() throws RosException {
//        MoveitMsgsPickupClient.cancelGoal();
//    }
//
//    public void cancelMoveitMsgsPickupGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
//        MoveitMsgsPickupClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
//    }
//
//    public moveit_msgs.PickupResult getMoveitMsgsPickupResult() throws RosException {
//        return MoveitMsgsPickupClient.getResult();
//    }
//
//    public SimpleClientGoalState getMoveitMsgsPickupState() {
//        return MoveitMsgsPickupClient.getState();
//    }
//
//    public void sendMoveitMsgsPickupGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PickupGoal goal) throws RosException {
//         MoveitMsgsPickupClient.sendGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PickupGoal.toRos(goal, node));
//    }
//
//    public void waitForMoveitMsgsPickupResult() throws InterruptedException {
//        MoveitMsgsPickupClient.waitForResult();
//    }
//
//    public boolean waitForMoveitMsgsPickupResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
//        return MoveitMsgsPickupClient.waitForResult(timeout, units);
//    }
//
//    public void waitForMoveitMsgsPickupServer() throws InterruptedException {
//        MoveitMsgsPickupClient.waitForServer();
//    }
//
//    public boolean waitForMoveitMsgsPickupServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
//        return MoveitMsgsPickupClient.waitForServer(timeout, units);
//    }
  public void cancelAllMoveitMsgsExecuteTrajectoryGoals() {
    MoveitMsgsExecuteTrajectoryClient.cancelAllGoals();
  }

  public void cancelMoveitMsgsExecuteTrajectoryGoal() throws RosException {
    MoveitMsgsExecuteTrajectoryClient.cancelGoal();
  }

  public void cancelMoveitMsgsExecuteTrajectoryGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    MoveitMsgsExecuteTrajectoryClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public moveit_msgs.ExecuteTrajectoryResult getMoveitMsgsExecuteTrajectoryResult() throws RosException {
    return MoveitMsgsExecuteTrajectoryClient.getResult();
  }

  public SimpleClientGoalState getMoveitMsgsExecuteTrajectoryState() {
    return MoveitMsgsExecuteTrajectoryClient.getState();
  }

  public void sendMoveitMsgsExecuteTrajectoryGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteTrajectoryGoal goal) throws RosException {
    log.debug("calling ros moveit execute");
    MoveitMsgsExecuteTrajectoryClient.sendGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteTrajectoryGoal.toRos(goal, node));
    log.debug("finished calling ros moveit execute");
  }

  public void waitForMoveitMsgsExecuteTrajectoryResult() throws InterruptedException {
    MoveitMsgsExecuteTrajectoryClient.waitForResult();
  }

  public boolean waitForMoveitMsgsExecuteTrajectoryResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveitMsgsExecuteTrajectoryClient.waitForResult(timeout, units);
  }

  public void waitForMoveitMsgsExecuteTrajectoryServer() throws InterruptedException {
    MoveitMsgsExecuteTrajectoryClient.waitForServer();
  }

  public boolean waitForMoveitMsgsExecuteTrajectoryServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveitMsgsExecuteTrajectoryClient.waitForServer(timeout, units);
  }

  public void cancelAllMoveitMsgsMoveGroupGoals() {
    MoveitMsgsMoveGroupClient.cancelAllGoals();
  }

  public void cancelMoveitMsgsMoveGroupGoal() throws RosException {
    MoveitMsgsMoveGroupClient.cancelGoal();
  }

  public void cancelMoveitMsgsMoveGroupGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    MoveitMsgsMoveGroupClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public moveit_msgs.MoveGroupResult getMoveitMsgsMoveGroupResult() throws RosException {
    return MoveitMsgsMoveGroupClient.getResult();
  }

  public SimpleClientGoalState getMoveitMsgsMoveGroupState() {
    return MoveitMsgsMoveGroupClient.getState();
  }

  public void sendMoveitMsgsMoveGroupGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.MoveGroupGoal goal) throws RosException {
    MoveitMsgsMoveGroupClient.sendGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.MoveGroupGoal.toRos(goal, node));
  }

  public void waitForMoveitMsgsMoveGroupResult() throws InterruptedException {
    MoveitMsgsMoveGroupClient.waitForResult();
  }

  public boolean waitForMoveitMsgsMoveGroupResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveitMsgsMoveGroupClient.waitForResult(timeout, units);
  }

  public void waitForMoveitMsgsMoveGroupServer() throws InterruptedException {
    MoveitMsgsMoveGroupClient.waitForServer();
  }

  public boolean waitForMoveitMsgsMoveGroupServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveitMsgsMoveGroupClient.waitForServer(timeout, units);
  }

  public void cancelAllMoveitMsgsPlaceGoals() {
    MoveitMsgsPlaceClient.cancelAllGoals();
  }

  public void cancelMoveitMsgsPlaceGoal() throws RosException {
    MoveitMsgsPlaceClient.cancelGoal();
  }

  public void cancelMoveitMsgsPlaceGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    MoveitMsgsPlaceClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public moveit_msgs.PlaceResult getMoveitMsgsPlaceResult() throws RosException {
    return MoveitMsgsPlaceClient.getResult();
  }

  public SimpleClientGoalState getMoveitMsgsPlaceState() {
    return MoveitMsgsPlaceClient.getState();
  }

  public void sendMoveitMsgsPlaceGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlaceGoal goal) throws RosException {
    MoveitMsgsPlaceClient.sendGoal(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlaceGoal.toRos(goal, node));
  }

  public void waitForMoveitMsgsPlaceResult() throws InterruptedException {
    MoveitMsgsPlaceClient.waitForResult();
  }

  public boolean waitForMoveitMsgsPlaceResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveitMsgsPlaceClient.waitForResult(timeout, units);
  }

  public void waitForMoveitMsgsPlaceServer() throws InterruptedException {
    MoveitMsgsPlaceClient.waitForServer();
  }

  public boolean waitForMoveitMsgsPlaceServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return MoveitMsgsPlaceClient.waitForServer(timeout, units);
  }

  public void cancelAllControlMsgsFollowJointTrajectoryGoals() {
    ControlMsgsFollowJointTrajectoryClient.cancelAllGoals();
  }

  public void cancelControlMsgsFollowJointTrajectoryGoal() throws RosException {
    ControlMsgsFollowJointTrajectoryClient.cancelGoal();
  }

  public void cancelControlMsgsFollowJointTrajectoryGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    ControlMsgsFollowJointTrajectoryClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public control_msgs.FollowJointTrajectoryResult getControlMsgsFollowJointTrajectoryResult() throws RosException {
    return ControlMsgsFollowJointTrajectoryClient.getResult();
  }

  public SimpleClientGoalState getControlMsgsFollowJointTrajectoryState() {
    return ControlMsgsFollowJointTrajectoryClient.getState();
  }

  public void sendControlMsgsFollowJointTrajectoryGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal goal) throws RosException {
    ControlMsgsFollowJointTrajectoryClient.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal.toRos(goal, node));
  }

  public void waitForControlMsgsFollowJointTrajectoryResult() throws InterruptedException {
    ControlMsgsFollowJointTrajectoryClient.waitForResult();
  }

  public boolean waitForControlMsgsFollowJointTrajectoryResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return ControlMsgsFollowJointTrajectoryClient.waitForResult(timeout, units);
  }

  public void waitForControlMsgsFollowJointTrajectoryServer() throws InterruptedException {
    ControlMsgsFollowJointTrajectoryClient.waitForServer();
  }

  public boolean waitForControlMsgsFollowJointTrajectoryServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return ControlMsgsFollowJointTrajectoryClient.waitForServer(timeout, units);
  }

  public void cancelAllControlMsgsGripperCommandGoals() {
    ControlMsgsGripperCommandClient.cancelAllGoals();
  }

  public void cancelControlMsgsGripperCommandGoal() throws RosException {
    ControlMsgsGripperCommandClient.cancelGoal();
  }

  public void cancelControlMsgsGripperCommandGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    ControlMsgsGripperCommandClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
  }

  public control_msgs.GripperCommandResult getControlMsgsGripperCommandResult() throws RosException {
    return ControlMsgsGripperCommandClient.getResult();
  }

  public SimpleClientGoalState getControlMsgsGripperCommandState() {
    return ControlMsgsGripperCommandClient.getState();
  }

  public void sendControlMsgsGripperCommandGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommandGoal goal) throws RosException {
    ControlMsgsGripperCommandClient.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommandGoal.toRos(goal, node));
  }

  public void waitForControlMsgsGripperCommandResult() throws InterruptedException {
    ControlMsgsGripperCommandClient.waitForResult();
  }

  public boolean waitForControlMsgsGripperCommandResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return ControlMsgsGripperCommandClient.waitForResult(timeout, units);
  }

  public void waitForControlMsgsGripperCommandServer() throws InterruptedException {
    ControlMsgsGripperCommandClient.waitForServer();
  }

  public boolean waitForControlMsgsGripperCommandServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    return ControlMsgsGripperCommandClient.waitForServer(timeout, units);
  }

  public boolean executeKinematicPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.RobotTrajectory rt) throws InterruptedException, RosException {
    edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteTrajectoryGoal goal = new edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteTrajectoryGoal(rt);
    sendMoveitMsgsExecuteTrajectoryGoal(goal);
    waitForMoveitMsgsExecuteTrajectoryResult(25, TimeUnit.SECONDS);
    SimpleClientGoalState state = getMoveitMsgsExecuteTrajectoryState();
    return (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
  }

  public boolean callExecuteKinematicPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteKnownTrajectoryRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteKnownTrajectoryResponse response) {
    try {
      return executeKinematicPath(request.getTrajectory());
    } catch (InterruptedException | RosException e) {
      e.printStackTrace();
      return false;
    }
  }

  @Override
  public void sendPointCloud(edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud2 msg) {
    System.err.println("This functionality is not implemented in the Melodic move group.");
  }
}

