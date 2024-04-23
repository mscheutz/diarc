/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.spot_ros;

import edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest;
import edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse;
import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RosException;
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
import sensor_msgs.CameraInfo;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SpotRos {
    final Object SpotClearBehaviorFaultLock = new Object();
    final Object SpotEstopReleaseLock = new Object();
    final Object SpotSelfRightLock = new Object();
    final Object SpotDockingStateLock = new Object();
    final Object SpotMaxVelocityLock = new Object();
    final Object SpotLocomotionModeLock = new Object();
    final Object SpotPowerOnLock = new Object();
    final Object SpotStopLock = new Object();
    final Object SpotStandLock = new Object();
    final Object SpotListGraphLock = new Object();
    final Object SpotClaimLock = new Object();
    final Object SpotPowerOffLock = new Object();
    final Object SpotSpotPoseLock = new Object();
    final Object SpotOpenDoorLock = new Object();
    final Object SpotUndockLock = new Object();
    final Object SpotReleaseLock = new Object();
    final Object SpotForceClaimLock = new Object();
    final Object SpotSitLock = new Object();
    final Object SpotEstopHardLock = new Object();
    final Object SpotStairModeLock = new Object();
    final Object SpotEstopGentleLock = new Object();
    final Object SpotDockLock = new Object();
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();
    private final Object SpotDepthFrontrightCameraInfoLock = new Object();
    private final Object SpotCameraBackCameraInfoLock = new Object();
    private final Object SpotCameraFrontleftImageLock = new Object();
    private final Object SpotCameraLeftImageLock = new Object();
    private final Object SpotDepthFrontleftImageLock = new Object();
    private final Object SpotStatusFeetLock = new Object();
    private final Object SpotDepthLeftCameraInfoLock = new Object();
    private final Object SpotStatusSystemFaultsLock = new Object();
    private final Object SpotOdometryTwistLock = new Object();
    private final Object SpotStatusBehaviorFaultsLock = new Object();
    private final Object SpotDepthFrontleftCameraInfoLock = new Object();
    private final Object SpotDepthLeftImageLock = new Object();
    private final Object SpotCameraBackImageLock = new Object();
    private final Object SpotCameraFrontleftCameraInfoLock = new Object();
    private final Object JointStatesLock = new Object();
    private final Object SpotCameraRightCameraInfoLock = new Object();
    private final Object SpotCameraFrontrightImageLock = new Object();
    private final Object SpotStatusWifiLock = new Object();
    private final Object TfStaticLock = new Object();
    private final Object SpotDepthRightImageLock = new Object();
    private final Object TfLock = new Object();
    private final Object SpotCameraRightImageLock = new Object();
    private final Object SpotDepthBackImageLock = new Object();
    private final Object SpotStatusMetricsLock = new Object();
    private final Object SpotCameraLeftCameraInfoLock = new Object();
    private final Object SpotStatusLeasesLock = new Object();
    private final Object SpotDepthFrontrightImageLock = new Object();
    private final Object SpotCameraFrontrightCameraInfoLock = new Object();
    private final Object SpotStatusMobilityParamsLock = new Object();
    private final Object SpotStatusBatteryStatesLock = new Object();
    private final Object SpotStatusPowerStateLock = new Object();
    private final Object SpotDepthRightCameraInfoLock = new Object();
    private final Object SpotOdometryLock = new Object();
    private final Object SpotDepthBackCameraInfoLock = new Object();
    private final Object SpotStatusEstopLock = new Object();
    // Services
    ServiceClient<spot_msgs.ClearBehaviorFaultRequest, spot_msgs.ClearBehaviorFaultResponse> SpotClearBehaviorFault;
    spot_msgs.ClearBehaviorFaultResponse SpotClearBehaviorFaultResponse;
    boolean SpotClearBehaviorFaultCondition = false;
    boolean SpotClearBehaviorFaultSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotEstopRelease;
    std_srvs.TriggerResponse SpotEstopReleaseResponse;
    boolean SpotEstopReleaseCondition = false;
    boolean SpotEstopReleaseSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotSelfRight;
    std_srvs.TriggerResponse SpotSelfRightResponse;
    boolean SpotSelfRightCondition = false;
    boolean SpotSelfRightSuccess = false;
    ServiceClient<spot_msgs.GetDockStateRequest, spot_msgs.GetDockStateResponse> SpotDockingState;
    spot_msgs.GetDockStateResponse SpotDockingStateResponse;
    boolean SpotDockingStateCondition = false;
    boolean SpotDockingStateSuccess = false;
    ServiceClient<spot_msgs.SetVelocityRequest, spot_msgs.SetVelocityResponse> SpotMaxVelocity;
    spot_msgs.SetVelocityResponse SpotMaxVelocityResponse;
    boolean SpotMaxVelocityCondition = false;
    boolean SpotMaxVelocitySuccess = false;
    ServiceClient<spot_msgs.SetLocomotionRequest, spot_msgs.SetLocomotionResponse> SpotLocomotionMode;
    spot_msgs.SetLocomotionResponse SpotLocomotionModeResponse;
    boolean SpotLocomotionModeCondition = false;
    boolean SpotLocomotionModeSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotPowerOn;
    std_srvs.TriggerResponse SpotPowerOnResponse;
    boolean SpotPowerOnCondition = false;
    boolean SpotPowerOnSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotStop;
    std_srvs.TriggerResponse SpotStopResponse;
    boolean SpotStopCondition = false;
    boolean SpotStopSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotStand;
    std_srvs.TriggerResponse SpotStandResponse;
    boolean SpotStandCondition = false;
    boolean SpotStandSuccess = false;
    ServiceClient<spot_msgs.ListGraphRequest, spot_msgs.ListGraphResponse> SpotListGraph;
    spot_msgs.ListGraphResponse SpotListGraphResponse;
    boolean SpotListGraphCondition = false;
    boolean SpotListGraphSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotClaim;
    std_srvs.TriggerResponse SpotClaimResponse;
    boolean SpotClaimCondition = false;
    boolean SpotClaimSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotPowerOff;
    std_srvs.TriggerResponse SpotPowerOffResponse;
    boolean SpotPowerOffCondition = false;
    boolean SpotPowerOffSuccess = false;
//    ServiceClient<spot_msgs.SpotPoseRequest, spot_msgs.SpotPoseResponse> SpotSpotPose;
//    spot_msgs.SpotPoseResponse SpotSpotPoseResponse;
    boolean SpotSpotPoseCondition = false;
    boolean SpotSpotPoseSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotOpenDoor;
    std_srvs.TriggerResponse SpotOpenDoorResponse;
    boolean SpotOpenDoorCondition = false;
    boolean SpotOpenDoorSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotUndock;
    std_srvs.TriggerResponse SpotUndockResponse;
    boolean SpotUndockCondition = false;
    boolean SpotUndockSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotRelease;
    std_srvs.TriggerResponse SpotReleaseResponse;
    boolean SpotReleaseCondition = false;
    boolean SpotReleaseSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotForceClaim;
    std_srvs.TriggerResponse SpotForceClaimResponse;
    boolean SpotForceClaimCondition = false;
    boolean SpotForceClaimSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotSit;
    std_srvs.TriggerResponse SpotSitResponse;
    boolean SpotSitCondition = false;
    boolean SpotSitSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotEstopHard;
    std_srvs.TriggerResponse SpotEstopHardResponse;
    boolean SpotEstopHardCondition = false;
    boolean SpotEstopHardSuccess = false;
    ServiceClient<std_srvs.SetBoolRequest, std_srvs.SetBoolResponse> SpotStairMode;
    std_srvs.SetBoolResponse SpotStairModeResponse;
    boolean SpotStairModeCondition = false;
    boolean SpotStairModeSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotEstopGentle;
    std_srvs.TriggerResponse SpotEstopGentleResponse;
    boolean SpotEstopGentleCondition = false;
    boolean SpotEstopGentleSuccess = false;
    ServiceClient<spot_msgs.ArmJointMovementRequest, spot_msgs.ArmJointMovementResponse> SpotArmJointMove;
    spot_msgs.ArmJointMovementResponse SpotArmJointMoveResponse;
    final Object SpotArmJointMoveLock = new Object();
    boolean SpotArmJointMoveCondition = false;
    boolean SpotArmJointMoveSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotArmCarry;
    final Object SpotArmCarryLock = new Object();
    boolean SpotArmCarryCondition = false;
    boolean SpotArmCarrySuccess = false;
    std_srvs.TriggerResponse SpotArmCarryResponse;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotArmStow;
    std_srvs.TriggerResponse SpotArmStowResponse;
    final Object SpotArmStowLock = new Object();
    boolean SpotArmStowCondition = false;
    boolean SpotArmStowSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotArmUnstow;
    std_srvs.TriggerResponse SpotArmUnstowResponse;
    final Object SpotArmUnstowLock = new Object();
    boolean SpotArmUnstowCondition = false;
    boolean SpotArmUnstowSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotGripperOpen;
    std_srvs.TriggerResponse SpotGripperOpenResponse;
    final Object SpotGripperOpenLock = new Object();
    boolean SpotGripperOpenCondition = false;
    boolean SpotGripperOpenSuccess = false;
    ServiceClient<std_srvs.TriggerRequest, std_srvs.TriggerResponse> SpotGripperClose;
    std_srvs.TriggerResponse SpotGripperCloseResponse;
    final Object SpotGripperCloseLock = new Object();
    boolean SpotGripperCloseCondition = false;
    boolean SpotGripperCloseSuccess = false;
    ServiceClient<spot_msgs.DockRequest, spot_msgs.DockResponse> SpotDock;
    spot_msgs.DockResponse SpotDockResponse;
    boolean SpotDockCondition = false;
    boolean SpotDockSuccess = false;
    // ROS connection
    private ConnectedNode node;
    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    // Subscription local data & locks
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotDepthFrontrightCameraInfo;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotCameraBackCameraInfo;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotCameraFrontleftImage;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotCameraLeftImage;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotDepthFrontleftImage;
    private edu.tufts.hrilab.diarcros.msg.spot_msgs.FootStateArray SpotStatusFeet;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotDepthLeftCameraInfo;
    private edu.tufts.hrilab.diarcros.msg.spot_msgs.SystemFaultState SpotStatusSystemFaults;
    private edu.tufts.hrilab.diarcros.msg.geometry_msgs.TwistWithCovarianceStamped SpotOdometryTwist;
    private edu.tufts.hrilab.diarcros.msg.spot_msgs.BehaviorFaultState SpotStatusBehaviorFaults;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotDepthFrontleftCameraInfo;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotDepthLeftImage;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotCameraBackImage;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotCameraFrontleftCameraInfo;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState JointStates;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotCameraRightCameraInfo;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotCameraFrontrightImage;
    //private edu.tufts.hrilab.diarcros.msg.spot_msgs.WiFiState SpotStatusWifi;
    //private edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage TfStatic;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotDepthRightImage;
    //private edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage Tf;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotCameraRightImage;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotDepthBackImage;
    //private edu.tufts.hrilab.diarcros.msg.spot_msgs.Metrics SpotStatusMetrics;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotCameraLeftCameraInfo;
    //private edu.tufts.hrilab.diarcros.msg.spot_msgs.LeaseArray SpotStatusLeases;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image SpotDepthFrontrightImage;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotCameraFrontrightCameraInfo;
    private edu.tufts.hrilab.diarcros.msg.spot_msgs.MobilityParams SpotStatusMobilityParams;
    private edu.tufts.hrilab.diarcros.msg.spot_msgs.BatteryStateArray SpotStatusBatteryStates;
    private edu.tufts.hrilab.diarcros.msg.spot_msgs.PowerState SpotStatusPowerState;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotDepthRightCameraInfo;
    //private edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry SpotOdometry;
    //private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo SpotDepthBackCameraInfo;
    private edu.tufts.hrilab.diarcros.msg.spot_msgs.EStopStateArray SpotStatusEstop;
    // Publishers
    //private Publisher<geometry_msgs.Pose> SpotBodyPosePublisher;
    private Publisher<geometry_msgs.Twist> SpotCmdVelPublisher;

    // Action Client
    // private SimpleActionClient<spot_msgs.TrajectoryActionFeedback, spot_msgs.TrajectoryActionGoal, spot_msgs.TrajectoryActionResult, spot_msgs.TrajectoryFeedback, spot_msgs.TrajectoryGoal, spot_msgs.TrajectoryResult> SpotMsgsTrajectoryClient;
    // private SimpleActionClient<spot_msgs.OpenDoorActionFeedback, spot_msgs.OpenDoorActionGoal, spot_msgs.OpenDoorActionResult, spot_msgs.OpenDoorFeedback, spot_msgs.OpenDoorGoal, spot_msgs.OpenDoorResult> SpotMsgsOpenDoorClient;
    // private SimpleActionClient<spot_msgs.NavigateToActionFeedback, spot_msgs.NavigateToActionGoal, spot_msgs.NavigateToActionResult, spot_msgs.NavigateToFeedback, spot_msgs.NavigateToGoal, spot_msgs.NavigateToResult> SpotMsgsNavigateToClient;

    public SpotRos(String graphname) {
        init(graphname);
    }

    public SpotRos() {
        init("ade/spot/spot_ros");
    }

    public void init(String graphname) {
        NodeMain nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of(graphname);
            }

            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                // Subscribers
                //Subscriber<CameraInfo> SpotDepthFrontrightCameraInfoSub = node.newSubscriber("/spot/depth/frontright/camera_info", CameraInfo._TYPE);
                //SpotDepthFrontrightCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthFrontrightCameraInfoLock) {
                //        SpotDepthFrontrightCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                //Subscriber<CameraInfo> SpotCameraBackCameraInfoSub = node.newSubscriber("/spot/camera/back/camera_info", CameraInfo._TYPE);
                //SpotCameraBackCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraBackCameraInfoLock) {
                //        SpotCameraBackCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotCameraFrontleftImageSub = node.newSubscriber("/spot/camera/frontleft/image", sensor_msgs.Image._TYPE);
                //SpotCameraFrontleftImageSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraFrontleftImageLock) {
                //        SpotCameraFrontleftImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotCameraLeftImageSub = node.newSubscriber("/spot/camera/left/image", sensor_msgs.Image._TYPE);
                //SpotCameraLeftImageSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraLeftImageLock) {
                //        SpotCameraLeftImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotDepthFrontleftImageSub = node.newSubscriber("/spot/depth/frontleft/image", sensor_msgs.Image._TYPE);
                //SpotDepthFrontleftImageSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthFrontleftImageLock) {
                //        SpotDepthFrontleftImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                Subscriber<spot_msgs.FootStateArray> SpotStatusFeetSub = node.newSubscriber("/spot/status/feet", spot_msgs.FootStateArray._TYPE);
                SpotStatusFeetSub.addMessageListener(msg -> {
                    synchronized (SpotStatusFeetLock) {
                        SpotStatusFeet = edu.tufts.hrilab.diarcros.msg.spot_msgs.FootStateArray.toAde(msg);
                    }
                });
                //Subscriber<CameraInfo> SpotDepthLeftCameraInfoSub = node.newSubscriber("/spot/depth/left/camera_info", CameraInfo._TYPE);
                //SpotDepthLeftCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthLeftCameraInfoLock) {
                //        SpotDepthLeftCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                Subscriber<spot_msgs.SystemFaultState> SpotStatusSystemFaultsSub = node.newSubscriber("/spot/status/system_faults", spot_msgs.SystemFaultState._TYPE);
                SpotStatusSystemFaultsSub.addMessageListener(msg -> {
                    synchronized (SpotStatusSystemFaultsLock) {
                        SpotStatusSystemFaults = edu.tufts.hrilab.diarcros.msg.spot_msgs.SystemFaultState.toAde(msg);
                    }
                });
                Subscriber<geometry_msgs.TwistWithCovarianceStamped> SpotOdometryTwistSub = node.newSubscriber("/spot/odometry/twist", geometry_msgs.TwistWithCovarianceStamped._TYPE);
                SpotOdometryTwistSub.addMessageListener(msg -> {
                    synchronized (SpotOdometryTwistLock) {
                        SpotOdometryTwist = edu.tufts.hrilab.diarcros.msg.geometry_msgs.TwistWithCovarianceStamped.toAde(msg);
                    }
                });
                Subscriber<spot_msgs.BehaviorFaultState> SpotStatusBehaviorFaultsSub = node.newSubscriber("/spot/status/behavior_faults", spot_msgs.BehaviorFaultState._TYPE);
                SpotStatusBehaviorFaultsSub.addMessageListener(msg -> {
                    synchronized (SpotStatusBehaviorFaultsLock) {
                        SpotStatusBehaviorFaults = edu.tufts.hrilab.diarcros.msg.spot_msgs.BehaviorFaultState.toAde(msg);
                    }
                });
                //Subscriber<CameraInfo> SpotDepthFrontleftCameraInfoSub = node.newSubscriber("/spot/depth/frontleft/camera_info", CameraInfo._TYPE);
                //SpotDepthFrontleftCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthFrontleftCameraInfoLock) {
                //        SpotDepthFrontleftCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotDepthLeftImageSub = node.newSubscriber("/spot/depth/left/image", sensor_msgs.Image._TYPE);
                //SpotDepthLeftImageSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthLeftImageLock) {
                //        SpotDepthLeftImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotCameraBackImageSub = node.newSubscriber("/spot/camera/back/image", sensor_msgs.Image._TYPE);
                //SpotCameraBackImageSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraBackImageLock) {
                //        SpotCameraBackImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<CameraInfo> SpotCameraFrontleftCameraInfoSub = node.newSubscriber("/spot/camera/frontleft/camera_info", CameraInfo._TYPE);
                //SpotCameraFrontleftCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraFrontleftCameraInfoLock) {
                //        SpotCameraFrontleftCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.JointState> JointStatesSub = node.newSubscriber("/joint_states", sensor_msgs.JointState._TYPE);
                //JointStatesSub.addMessageListener(msg -> {
                //    synchronized (JointStatesLock) {
                //        JointStates = edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toAde(msg);
                //    }
                //});
                //Subscriber<CameraInfo> SpotCameraRightCameraInfoSub = node.newSubscriber("/spot/camera/right/camera_info", CameraInfo._TYPE);
                //SpotCameraRightCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraRightCameraInfoLock) {
                //        SpotCameraRightCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotCameraFrontrightImageSub = node.newSubscriber("/spot/camera/frontright/image", sensor_msgs.Image._TYPE);
                //SpotCameraFrontrightImageSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraFrontrightImageLock) {
                //        SpotCameraFrontrightImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<spot_msgs.WiFiState> SpotStatusWifiSub = node.newSubscriber("/spot/status/wifi", spot_msgs.WiFiState._TYPE);
                //SpotStatusWifiSub.addMessageListener(msg -> {
                //    synchronized (SpotStatusWifiLock) {
                //        SpotStatusWifi = edu.tufts.hrilab.diarcros.msg.spot_msgs.WiFiState.toAde(msg);
                //    }
                //});
                //Subscriber<tf2_msgs.TFMessage> TfStaticSub = node.newSubscriber("/tf_static", tf2_msgs.TFMessage._TYPE);
                //TfStaticSub.addMessageListener(msg -> {
                //    synchronized (TfStaticLock) {
                //        TfStatic = edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotDepthRightImageSub = node.newSubscriber("/spot/depth/right/image", sensor_msgs.Image._TYPE);
                //SpotDepthRightImageSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthRightImageLock) {
                //        SpotDepthRightImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<tf2_msgs.TFMessage> TfSub = node.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
                //TfSub.addMessageListener(msg -> {
                //    synchronized (TfLock) {
                //        Tf = edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotCameraRightImageSub = node.newSubscriber("/spot/camera/right/image", sensor_msgs.Image._TYPE);
                //SpotCameraRightImageSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraRightImageLock) {
                //        SpotCameraRightImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotDepthBackImageSub = node.newSubscriber("/spot/depth/back/image", sensor_msgs.Image._TYPE);
                //SpotDepthBackImageSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthBackImageLock) {
                //        SpotDepthBackImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<spot_msgs.Metrics> SpotStatusMetricsSub = node.newSubscriber("/spot/status/metrics", spot_msgs.Metrics._TYPE);
                //SpotStatusMetricsSub.addMessageListener(msg -> {
                //    synchronized (SpotStatusMetricsLock) {
                //        SpotStatusMetrics = edu.tufts.hrilab.diarcros.msg.spot_msgs.Metrics.toAde(msg);
                //    }
                //});
                //Subscriber<CameraInfo> SpotCameraLeftCameraInfoSub = node.newSubscriber("/spot/camera/left/camera_info", CameraInfo._TYPE);
                //SpotCameraLeftCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraLeftCameraInfoLock) {
                //        SpotCameraLeftCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                //Subscriber<spot_msgs.LeaseArray> SpotStatusLeasesSub = node.newSubscriber("/spot/status/leases", spot_msgs.LeaseArray._TYPE);
                //SpotStatusLeasesSub.addMessageListener(msg -> {
                //    synchronized (SpotStatusLeasesLock) {
                //        SpotStatusLeases = edu.tufts.hrilab.diarcros.msg.spot_msgs.LeaseArray.toAde(msg);
                //    }
                //});
                //Subscriber<sensor_msgs.Image> SpotDepthFrontrightImageSub = node.newSubscriber("/spot/depth/frontright/image", sensor_msgs.Image._TYPE);
                //SpotDepthFrontrightImageSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthFrontrightImageLock) {
                //        SpotDepthFrontrightImage = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
                //    }
                //});
                //Subscriber<CameraInfo> SpotCameraFrontrightCameraInfoSub = node.newSubscriber("/spot/camera/frontright/camera_info", CameraInfo._TYPE);
                //SpotCameraFrontrightCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotCameraFrontrightCameraInfoLock) {
                //        SpotCameraFrontrightCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                Subscriber<spot_msgs.MobilityParams> SpotStatusMobilityParamsSub = node.newSubscriber("/spot/status/mobility_params", spot_msgs.MobilityParams._TYPE);
                SpotStatusMobilityParamsSub.addMessageListener(msg -> {
                    synchronized (SpotStatusMobilityParamsLock) {
                        SpotStatusMobilityParams = edu.tufts.hrilab.diarcros.msg.spot_msgs.MobilityParams.toAde(msg);
                    }
                });
                Subscriber<spot_msgs.BatteryStateArray> SpotStatusBatteryStatesSub = node.newSubscriber("/spot/status/battery_states", spot_msgs.BatteryStateArray._TYPE);
                SpotStatusBatteryStatesSub.addMessageListener(msg -> {
                    synchronized (SpotStatusBatteryStatesLock) {
                        SpotStatusBatteryStates = edu.tufts.hrilab.diarcros.msg.spot_msgs.BatteryStateArray.toAde(msg);
                    }
                });
                Subscriber<spot_msgs.PowerState> SpotStatusPowerStateSub = node.newSubscriber("/spot/status/power_state", spot_msgs.PowerState._TYPE);
                SpotStatusPowerStateSub.addMessageListener(msg -> {
                    synchronized (SpotStatusPowerStateLock) {
                        SpotStatusPowerState = edu.tufts.hrilab.diarcros.msg.spot_msgs.PowerState.toAde(msg);
                    }
                });
                //Subscriber<CameraInfo> SpotDepthRightCameraInfoSub = node.newSubscriber("/spot/depth/right/camera_info", CameraInfo._TYPE);
                //SpotDepthRightCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthRightCameraInfoLock) {
                //        SpotDepthRightCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                //Subscriber<nav_msgs.Odometry> SpotOdometrySub = node.newSubscriber("/spot/odometry", nav_msgs.Odometry._TYPE);
                //SpotOdometrySub.addMessageListener(msg -> {
                //    synchronized (SpotOdometryLock) {
                //        SpotOdometry = edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry.toAde(msg);
                //    }
                //});
                //Subscriber<CameraInfo> SpotDepthBackCameraInfoSub = node.newSubscriber("/spot/depth/back/camera_info", CameraInfo._TYPE);
                //SpotDepthBackCameraInfoSub.addMessageListener(msg -> {
                //    synchronized (SpotDepthBackCameraInfoLock) {
                //        SpotDepthBackCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
                //    }
                //});
                Subscriber<spot_msgs.EStopStateArray> SpotStatusEstopSub = node.newSubscriber("/spot/status/estop", spot_msgs.EStopStateArray._TYPE);
                SpotStatusEstopSub.addMessageListener(msg -> {
                    synchronized (SpotStatusEstopLock) {
                        SpotStatusEstop = edu.tufts.hrilab.diarcros.msg.spot_msgs.EStopStateArray.toAde(msg);
                    }
                });
                // Publishers
                //SpotBodyPosePublisher = node.newPublisher("/spot/body_pose", geometry_msgs.Pose._TYPE);
                SpotCmdVelPublisher = node.newPublisher("/spot/cmd_vel", geometry_msgs.Twist._TYPE);
                // Services
                try {
                    SpotClearBehaviorFault = node.newServiceClient("/spot/clear_behavior_fault", spot_msgs.ClearBehaviorFault._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotEstopRelease = node.newServiceClient("/spot/estop/release", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotSelfRight = node.newServiceClient("/spot/self_right", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotDockingState = node.newServiceClient("/spot/docking_state", spot_msgs.GetDockState._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotMaxVelocity = node.newServiceClient("/spot/max_velocity", spot_msgs.SetVelocity._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotLocomotionMode = node.newServiceClient("/spot/locomotion_mode", spot_msgs.SetLocomotion._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotPowerOn = node.newServiceClient("/spot/power_on", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotStop = node.newServiceClient("/spot/stop", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotStand = node.newServiceClient("/spot/stand", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotListGraph = node.newServiceClient("/spot/list_graph", spot_msgs.ListGraph._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotClaim = node.newServiceClient("/spot/claim", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotPowerOff = node.newServiceClient("/spot/power_off", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
//                try {
//                    SpotSpotPose = node.newServiceClient("/spot/spot_pose", spot_msgs.SpotPose._TYPE);
//                } catch (org.ros.exception.ServiceNotFoundException e) {
//                    System.err.println("Could not find service! Exception: " + e);
//                }
//                try {
//                    SpotOpenDoor = node.newServiceClient("/spot/open_door", std_srvs.Trigger._TYPE);
//                } catch (org.ros.exception.ServiceNotFoundException e) {
//                    System.err.println("Could not find service! Exception: " + e);
//                }
                try {
                    SpotUndock = node.newServiceClient("/spot/undock", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotRelease = node.newServiceClient("/spot/release", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotForceClaim = node.newServiceClient("/spot/force_claim", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotSit = node.newServiceClient("/spot/sit", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotEstopHard = node.newServiceClient("/spot/estop/hard", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotStairMode = node.newServiceClient("/spot/stair_mode", std_srvs.SetBool._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotEstopGentle = node.newServiceClient("/spot/estop/gentle", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotArmJointMove = node.newServiceClient("/spot/arm_joint_move", spot_msgs.ArmJointMovement._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotArmCarry = node.newServiceClient("/spot/arm_carry", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotArmStow = node.newServiceClient("/spot/arm_stow", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotArmUnstow = node.newServiceClient("/spot/arm_unstow", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotGripperOpen = node.newServiceClient("/spot/gripper_open", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotGripperClose = node.newServiceClient("/spot/gripper_close", std_srvs.Trigger._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    SpotDock = node.newServiceClient("/spot/dock", spot_msgs.Dock._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                // Action Client
//                 try {
//                     SpotMsgsTrajectoryClient = new SimpleActionClient<>("/spot/spot_ros", new ActionSpec(spot_msgs.TrajectoryAction.class, "spot_msgs/TrajectoryAction", "spot_msgs/TrajectoryActionFeedback", "spot_msgs/TrajectoryActionGoal", "spot_msgs/TrajectoryActionResult", "spot_msgs/TrajectoryFeedback", "spot_msgs/TrajectoryGoal", "spot_msgs/TrajectoryResult"));

//                 } catch (RosException e) {
//                     e.printStackTrace();
//                 }
//                 while (SpotMsgsTrajectoryClient == null) try {
//                     Thread.sleep(100);
//                 } catch (InterruptedException e) {
//                     e.printStackTrace();
//                 }
//                 SpotMsgsTrajectoryClient.addClientPubSub(node);
//                 try {
//                     SpotMsgsOpenDoorClient = new SimpleActionClient<>("/spot/spot_ros", new ActionSpec(spot_msgs.OpenDoorAction.class, "spot_msgs/OpenDoorAction", "spot_msgs/OpenDoorActionFeedback", "spot_msgs/OpenDoorActionGoal", "spot_msgs/OpenDoorActionResult", "spot_msgs/OpenDoorFeedback", "spot_msgs/OpenDoorGoal", "spot_msgs/OpenDoorResult"));

//                 } catch (RosException e) {
//                     e.printStackTrace();
//                 }
//                 while (SpotMsgsOpenDoorClient == null) try {
//                     Thread.sleep(100);
//                 } catch (InterruptedException e) {
//                     e.printStackTrace();
//                 }
//                 SpotMsgsOpenDoorClient.addClientPubSub(node);
// ;
//                 try {
//                     SpotMsgsNavigateToClient = new SimpleActionClient<>("/spot/spot_ros", new ActionSpec(spot_msgs.NavigateToAction.class, "spot_msgs/NavigateToAction", "spot_msgs/NavigateToActionFeedback", "spot_msgs/NavigateToActionGoal", "spot_msgs/NavigateToActionResult", "spot_msgs/NavigateToFeedback", "spot_msgs/NavigateToGoal", "spot_msgs/NavigateToResult"));

//                 } catch (RosException e) {
//                     e.printStackTrace();
//                 }
//                 while (SpotMsgsNavigateToClient == null) try {
//                     Thread.sleep(100);
//                 } catch (InterruptedException e) {
//                     e.printStackTrace();
//                 }
//                 SpotMsgsNavigateToClient.addClientPubSub(node);
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
            NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
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
    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotDepthFrontrightCameraInfo() {
    //    return SpotDepthFrontrightCameraInfo;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotCameraBackCameraInfo() {
    //    return SpotCameraBackCameraInfo;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotCameraFrontleftImage() {
    //    return SpotCameraFrontleftImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotCameraLeftImage() {
    //    return SpotCameraLeftImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotDepthFrontleftImage() {
    //    return SpotDepthFrontleftImage;
    //}

    public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.FootStateArray getSpotStatusFeet() {
        return SpotStatusFeet;
    }

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotDepthLeftCameraInfo() {
    //    return SpotDepthLeftCameraInfo;
    //}

    public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.SystemFaultState getSpotStatusSystemFaults() {
        return SpotStatusSystemFaults;
    }

    public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.TwistWithCovarianceStamped getSpotOdometryTwist() {
        return SpotOdometryTwist;
    }

    public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.BehaviorFaultState getSpotStatusBehaviorFaults() {
        return SpotStatusBehaviorFaults;
    }

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotDepthFrontleftCameraInfo() {
    //    return SpotDepthFrontleftCameraInfo;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotDepthLeftImage() {
    //    return SpotDepthLeftImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotCameraBackImage() {
    //    return SpotCameraBackImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotCameraFrontleftCameraInfo() {
    //    return SpotCameraFrontleftCameraInfo;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getJointStates() {
    //    return JointStates;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotCameraRightCameraInfo() {
    //    return SpotCameraRightCameraInfo;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotCameraFrontrightImage() {
    //    return SpotCameraFrontrightImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.WiFiState getSpotStatusWifi() {
    //    return SpotStatusWifi;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage getTfStatic() {
    //    return TfStatic;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotDepthRightImage() {
    //    return SpotDepthRightImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage getTf() {
    //    return Tf;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotCameraRightImage() {
    //    return SpotCameraRightImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotDepthBackImage() {
    //    return SpotDepthBackImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.Metrics getSpotStatusMetrics() {
    //    return SpotStatusMetrics;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotCameraLeftCameraInfo() {
    //    return SpotCameraLeftCameraInfo;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.LeaseArray getSpotStatusLeases() {
    //    return SpotStatusLeases;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getSpotDepthFrontrightImage() {
    //    return SpotDepthFrontrightImage;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotCameraFrontrightCameraInfo() {
    //    return SpotCameraFrontrightCameraInfo;
    //}

    public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.MobilityParams getSpotStatusMobilityParams() {
        return SpotStatusMobilityParams;
    }

    public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.BatteryStateArray getSpotStatusBatteryStates() {
        return SpotStatusBatteryStates;
    }

    public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.PowerState getSpotStatusPowerState() {
        return SpotStatusPowerState;
    }

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotDepthRightCameraInfo() {
    //    return SpotDepthRightCameraInfo;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry getSpotOdometry() {
    //    return SpotOdometry;
    //}

    //public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getSpotDepthBackCameraInfo() {
    //    return SpotDepthBackCameraInfo;
    //}

    public synchronized edu.tufts.hrilab.diarcros.msg.spot_msgs.EStopStateArray getSpotStatusEstop() {
        return SpotStatusEstop;
    }

    // Publishers
    //public void sendSpotBodyPose(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose msg) {
    //    SpotBodyPosePublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose.toRos(msg, node));
    //}

    public void sendSpotCmdVel(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist msg) {
        SpotCmdVelPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist.toRos(msg, node));
    }

    public boolean callSpotArmJointMove(edu.tufts.hrilab.diarcros.msg.spot_msgs.ArmJointMovementRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.ArmJointMovementResponse response) {
        SpotArmJointMoveCondition = false;
        SpotArmJointMoveSuccess = false;
        SpotArmJointMove.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.ArmJointMovementRequest.toRos(request, node),
                new ServiceResponseListener<spot_msgs.ArmJointMovementResponse>() {

                    @Override
                    public void onSuccess(spot_msgs.ArmJointMovementResponse mt) {
                        SpotArmJointMoveResponse = mt;
                        synchronized(SpotArmJointMoveLock) {
                            SpotArmJointMoveCondition = true;
                            SpotArmJointMoveSuccess = true;
                            SpotArmJointMoveLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(SpotArmJointMoveLock) {
                            SpotArmJointMoveCondition = true;
                            SpotArmJointMoveSuccess = false;
                            SpotArmJointMoveLock.notify();
                        }
                    }
                });

        synchronized(SpotArmJointMoveLock) {
            while(!SpotArmJointMoveCondition) {
                try {
                    SpotArmJointMoveLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotArmJointMoveSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.ArmJointMovementResponse.toAde(SpotArmJointMoveResponse, response);
        }
        return SpotArmJointMoveSuccess;
    }
    public boolean callSpotArmCarry(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotArmCarryCondition = false;
        SpotArmCarrySuccess = false;
        SpotArmCarry.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node),
                new ServiceResponseListener<std_srvs.TriggerResponse>() {

                    @Override
                    public void onSuccess(std_srvs.TriggerResponse mt) {
                        SpotArmCarryResponse = mt;
                        synchronized(SpotArmCarryLock) {
                            SpotArmCarryCondition = true;
                            SpotArmCarrySuccess = true;
                            SpotArmCarryLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(SpotArmCarryLock) {
                            SpotArmCarryCondition = true;
                            SpotArmCarrySuccess = false;
                            SpotArmCarryLock.notify();
                        }
                    }
                });

        synchronized(SpotArmCarryLock) {
            while(!SpotArmCarryCondition) {
                try {
                    SpotArmCarryLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotArmCarrySuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotArmCarryResponse, response);
        }
        return SpotArmCarrySuccess;
    }

    public boolean callSpotArmStow(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotArmStowCondition = false;
        SpotArmStowSuccess = false;
        SpotArmStow.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node),
                new ServiceResponseListener<std_srvs.TriggerResponse>() {

                    @Override
                    public void onSuccess(std_srvs.TriggerResponse mt) {
                        SpotArmStowResponse = mt;
                        synchronized(SpotArmStowLock) {
                            SpotArmStowCondition = true;
                            SpotArmStowSuccess = true;
                            SpotArmStowLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(SpotArmStowLock) {
                            SpotArmStowCondition = true;
                            SpotArmStowSuccess = false;
                            SpotArmStowLock.notify();
                        }
                    }
                });
        synchronized(SpotArmStowLock) {
            while(!SpotArmStowCondition) {
                try {
                    SpotArmStowLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotArmStowSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotArmStowResponse, response);
        }
        return SpotArmStowSuccess;
    }
    public boolean callSpotArmUnstow(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotArmUnstowCondition = false;
        SpotArmUnstowSuccess = false;
        SpotArmUnstow.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node),
                new ServiceResponseListener<std_srvs.TriggerResponse>() {

                    @Override
                    public void onSuccess(std_srvs.TriggerResponse mt) {
                        SpotArmUnstowResponse = mt;
                        synchronized(SpotArmUnstowLock) {
                            SpotArmUnstowCondition = true;
                            SpotArmUnstowSuccess = true;
                            SpotArmUnstowLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(SpotArmUnstowLock) {
                            SpotArmUnstowCondition = true;
                            SpotArmUnstowSuccess = false;
                            SpotArmUnstowLock.notify();
                        }
                    }
                });

        synchronized(SpotArmUnstowLock) {
            while(!SpotArmUnstowCondition) {
                try {
                    SpotArmUnstowLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotArmUnstowSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotArmUnstowResponse, response);
        }
        return SpotArmUnstowSuccess;
    }
    public boolean callSpotGripperOpen(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotGripperOpenCondition = false;
        SpotGripperOpenSuccess = false;
        SpotGripperOpen.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node),
                new ServiceResponseListener<std_srvs.TriggerResponse>() {

                    @Override
                    public void onSuccess(std_srvs.TriggerResponse mt) {
                        SpotGripperOpenResponse = mt;
                        synchronized(SpotGripperOpenLock) {
                            SpotGripperOpenCondition = true;
                            SpotGripperOpenSuccess = true;
                            SpotGripperOpenLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(SpotGripperOpenLock) {
                            SpotGripperOpenCondition = true;
                            SpotGripperOpenSuccess = false;
                            SpotGripperOpenLock.notify();
                        }
                    }
                });

        synchronized(SpotGripperOpenLock) {
            while(!SpotGripperOpenCondition) {
                try {
                    SpotGripperOpenLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotGripperOpenSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotGripperOpenResponse, response);
        }
        return SpotGripperOpenSuccess;
    }
    public boolean callSpotGripperClose(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotGripperCloseCondition = false;
        SpotGripperCloseSuccess = false;
        SpotGripperClose.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node),
                new ServiceResponseListener<std_srvs.TriggerResponse>() {

                    @Override
                    public void onSuccess(std_srvs.TriggerResponse mt) {
                        SpotGripperCloseResponse = mt;
                        synchronized(SpotGripperCloseLock) {
                            SpotGripperCloseCondition = true;
                            SpotGripperCloseSuccess = true;
                            SpotGripperCloseLock.notify();
                        }
                    }

                    @Override
                    public void onFailure(org.ros.exception.RemoteException re) {
                        synchronized(SpotGripperCloseLock) {
                            SpotGripperCloseCondition = true;
                            SpotGripperCloseSuccess = false;
                            SpotGripperCloseLock.notify();
                        }
                    }
                });

        synchronized(SpotGripperCloseLock) {
            while(!SpotGripperCloseCondition) {
                try {
                    SpotGripperCloseLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotGripperCloseSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotGripperCloseResponse, response);
        }
        return SpotGripperCloseSuccess;
    }
    // Services
    public boolean callSpotClearBehaviorFault(edu.tufts.hrilab.diarcros.msg.spot_msgs.ClearBehaviorFaultRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.ClearBehaviorFaultResponse response) {
        SpotClearBehaviorFaultCondition = false;
        SpotClearBehaviorFaultSuccess = false;
        SpotClearBehaviorFault.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.ClearBehaviorFaultRequest.toRos(request, node), new ServiceResponseListener<spot_msgs.ClearBehaviorFaultResponse>() {

            @Override
            public void onSuccess(spot_msgs.ClearBehaviorFaultResponse mt) {
                SpotClearBehaviorFaultResponse = mt;
                synchronized (SpotClearBehaviorFaultLock) {
                    SpotClearBehaviorFaultCondition = true;
                    SpotClearBehaviorFaultSuccess = true;
                    SpotClearBehaviorFaultLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotClearBehaviorFaultLock) {
                    SpotClearBehaviorFaultCondition = true;
                    SpotClearBehaviorFaultSuccess = false;
                    SpotClearBehaviorFaultLock.notify();
                }
            }
        });

        synchronized (SpotClearBehaviorFaultLock) {
            while (!SpotClearBehaviorFaultCondition) {
                try {
                    SpotClearBehaviorFaultLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotClearBehaviorFaultSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.ClearBehaviorFaultResponse.toAde(SpotClearBehaviorFaultResponse, response);
        }
        return SpotClearBehaviorFaultSuccess;
    }

    public boolean callSpotEstopRelease(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotEstopReleaseCondition = false;
        SpotEstopReleaseSuccess = false;
        SpotEstopRelease.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotEstopReleaseResponse = mt;
                synchronized (SpotEstopReleaseLock) {
                    SpotEstopReleaseCondition = true;
                    SpotEstopReleaseSuccess = true;
                    SpotEstopReleaseLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotEstopReleaseLock) {
                    SpotEstopReleaseCondition = true;
                    SpotEstopReleaseSuccess = false;
                    SpotEstopReleaseLock.notify();
                }
            }
        });

        synchronized (SpotEstopReleaseLock) {
            while (!SpotEstopReleaseCondition) {
                try {
                    SpotEstopReleaseLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotEstopReleaseSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotEstopReleaseResponse, response);
        }
        return SpotEstopReleaseSuccess;
    }

    public boolean callSpotSelfRight(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotSelfRightCondition = false;
        SpotSelfRightSuccess = false;
        SpotSelfRight.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotSelfRightResponse = mt;
                synchronized (SpotSelfRightLock) {
                    SpotSelfRightCondition = true;
                    SpotSelfRightSuccess = true;
                    SpotSelfRightLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotSelfRightLock) {
                    SpotSelfRightCondition = true;
                    SpotSelfRightSuccess = false;
                    SpotSelfRightLock.notify();
                }
            }
        });

        synchronized (SpotSelfRightLock) {
            while (!SpotSelfRightCondition) {
                try {
                    SpotSelfRightLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotSelfRightSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotSelfRightResponse, response);
        }
        return SpotSelfRightSuccess;
    }

    public boolean callSpotDockingState(edu.tufts.hrilab.diarcros.msg.spot_msgs.GetDockStateRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.GetDockStateResponse response) {
        SpotDockingStateCondition = false;
        SpotDockingStateSuccess = false;
        SpotDockingState.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.GetDockStateRequest.toRos(request, node), new ServiceResponseListener<spot_msgs.GetDockStateResponse>() {

            @Override
            public void onSuccess(spot_msgs.GetDockStateResponse mt) {
                SpotDockingStateResponse = mt;
                synchronized (SpotDockingStateLock) {
                    SpotDockingStateCondition = true;
                    SpotDockingStateSuccess = true;
                    SpotDockingStateLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotDockingStateLock) {
                    SpotDockingStateCondition = true;
                    SpotDockingStateSuccess = false;
                    SpotDockingStateLock.notify();
                }
            }
        });

        synchronized (SpotDockingStateLock) {
            while (!SpotDockingStateCondition) {
                try {
                    SpotDockingStateLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotDockingStateSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.GetDockStateResponse.toAde(SpotDockingStateResponse, response);
        }
        return SpotDockingStateSuccess;
    }

    public boolean callSpotMaxVelocity(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetVelocityRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.SetVelocityResponse response) {
        SpotMaxVelocityCondition = false;
        SpotMaxVelocitySuccess = false;
        SpotMaxVelocity.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetVelocityRequest.toRos(request, node), new ServiceResponseListener<spot_msgs.SetVelocityResponse>() {

            @Override
            public void onSuccess(spot_msgs.SetVelocityResponse mt) {
                SpotMaxVelocityResponse = mt;
                synchronized (SpotMaxVelocityLock) {
                    SpotMaxVelocityCondition = true;
                    SpotMaxVelocitySuccess = true;
                    SpotMaxVelocityLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotMaxVelocityLock) {
                    SpotMaxVelocityCondition = true;
                    SpotMaxVelocitySuccess = false;
                    SpotMaxVelocityLock.notify();
                }
            }
        });

        synchronized (SpotMaxVelocityLock) {
            while (!SpotMaxVelocityCondition) {
                try {
                    SpotMaxVelocityLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotMaxVelocitySuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.SetVelocityResponse.toAde(SpotMaxVelocityResponse, response);
        }
        return SpotMaxVelocitySuccess;
    }

    public boolean callSpotLocomotionMode(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetLocomotionRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.SetLocomotionResponse response) {
        SpotLocomotionModeCondition = false;
        SpotLocomotionModeSuccess = false;
        SpotLocomotionMode.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.SetLocomotionRequest.toRos(request, node), new ServiceResponseListener<spot_msgs.SetLocomotionResponse>() {

            @Override
            public void onSuccess(spot_msgs.SetLocomotionResponse mt) {
                SpotLocomotionModeResponse = mt;
                synchronized (SpotLocomotionModeLock) {
                    SpotLocomotionModeCondition = true;
                    SpotLocomotionModeSuccess = true;
                    SpotLocomotionModeLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotLocomotionModeLock) {
                    SpotLocomotionModeCondition = true;
                    SpotLocomotionModeSuccess = false;
                    SpotLocomotionModeLock.notify();
                }
            }
        });

        synchronized (SpotLocomotionModeLock) {
            while (!SpotLocomotionModeCondition) {
                try {
                    SpotLocomotionModeLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotLocomotionModeSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.SetLocomotionResponse.toAde(SpotLocomotionModeResponse, response);
        }
        return SpotLocomotionModeSuccess;
    }

    public boolean callSpotPowerOn(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotPowerOnCondition = false;
        SpotPowerOnSuccess = false;
        SpotPowerOn.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotPowerOnResponse = mt;
                synchronized (SpotPowerOnLock) {
                    SpotPowerOnCondition = true;
                    SpotPowerOnSuccess = true;
                    SpotPowerOnLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotPowerOnLock) {
                    SpotPowerOnCondition = true;
                    SpotPowerOnSuccess = false;
                    SpotPowerOnLock.notify();
                }
            }
        });

        synchronized (SpotPowerOnLock) {
            while (!SpotPowerOnCondition) {
                try {
                    SpotPowerOnLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotPowerOnSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotPowerOnResponse, response);
        }
        return SpotPowerOnSuccess;
    }

    public boolean callSpotStop(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotStopCondition = false;
        SpotStopSuccess = false;
        SpotStop.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotStopResponse = mt;
                synchronized (SpotStopLock) {
                    SpotStopCondition = true;
                    SpotStopSuccess = true;
                    SpotStopLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotStopLock) {
                    SpotStopCondition = true;
                    SpotStopSuccess = false;
                    SpotStopLock.notify();
                }
            }
        });

        synchronized (SpotStopLock) {
            while (!SpotStopCondition) {
                try {
                    SpotStopLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotStopSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotStopResponse, response);
        }
        return SpotStopSuccess;
    }

    public boolean callSpotStand() {
        SpotStandCondition = false;
        SpotStandSuccess = false;
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request = new TriggerRequest();
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response = new TriggerResponse();
        SpotStand.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotStandResponse = mt;
                synchronized (SpotStandLock) {
                    SpotStandCondition = true;
                    SpotStandSuccess = true;
                    SpotStandLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotStandLock) {
                    SpotStandCondition = true;
                    SpotStandSuccess = false;
                    SpotStandLock.notify();
                }
            }
        });

        synchronized (SpotStandLock) {
            while (!SpotStandCondition) {
                try {
                    SpotStandLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotStandSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotStandResponse, response);
        }
        return SpotStandSuccess;
    }

    public boolean callSpotListGraph(edu.tufts.hrilab.diarcros.msg.spot_msgs.ListGraphRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.ListGraphResponse response) {
        SpotListGraphCondition = false;
        SpotListGraphSuccess = false;
        SpotListGraph.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.ListGraphRequest.toRos(request, node), new ServiceResponseListener<spot_msgs.ListGraphResponse>() {

            @Override
            public void onSuccess(spot_msgs.ListGraphResponse mt) {
                SpotListGraphResponse = mt;
                synchronized (SpotListGraphLock) {
                    SpotListGraphCondition = true;
                    SpotListGraphSuccess = true;
                    SpotListGraphLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotListGraphLock) {
                    SpotListGraphCondition = true;
                    SpotListGraphSuccess = false;
                    SpotListGraphLock.notify();
                }
            }
        });

        synchronized (SpotListGraphLock) {
            while (!SpotListGraphCondition) {
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

    public boolean callSpotClaim(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotClaimCondition = false;
        SpotClaimSuccess = false;
        SpotClaim.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotClaimResponse = mt;
                synchronized (SpotClaimLock) {
                    SpotClaimCondition = true;
                    SpotClaimSuccess = true;
                    SpotClaimLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotClaimLock) {
                    SpotClaimCondition = true;
                    SpotClaimSuccess = false;
                    SpotClaimLock.notify();
                }
            }
        });

        synchronized (SpotClaimLock) {
            while (!SpotClaimCondition) {
                try {
                    SpotClaimLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotClaimSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotClaimResponse, response);
        }
        return SpotClaimSuccess;
    }

    public boolean callSpotPowerOff(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotPowerOffCondition = false;
        SpotPowerOffSuccess = false;
        SpotPowerOff.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotPowerOffResponse = mt;
                synchronized (SpotPowerOffLock) {
                    SpotPowerOffCondition = true;
                    SpotPowerOffSuccess = true;
                    SpotPowerOffLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotPowerOffLock) {
                    SpotPowerOffCondition = true;
                    SpotPowerOffSuccess = false;
                    SpotPowerOffLock.notify();
                }
            }
        });

        synchronized (SpotPowerOffLock) {
            while (!SpotPowerOffCondition) {
                try {
                    SpotPowerOffLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotPowerOffSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotPowerOffResponse, response);
        }
        return SpotPowerOffSuccess;
    }

//    public boolean callSpotSpotPose(edu.tufts.hrilab.diarcros.msg.spot_msgs.SpotPoseRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.SpotPoseResponse response) {
//        SpotSpotPoseCondition = false;
//        SpotSpotPoseSuccess = false;
//        SpotSpotPose.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.SpotPoseRequest.toRos(request, node), new ServiceResponseListener<spot_msgs.SpotPoseResponse>() {
//
//            @Override
//            public void onSuccess(spot_msgs.SpotPoseResponse mt) {
//                SpotSpotPoseResponse = mt;
//                synchronized (SpotSpotPoseLock) {
//                    SpotSpotPoseCondition = true;
//                    SpotSpotPoseSuccess = true;
//                    SpotSpotPoseLock.notify();
//                }
//            }
//
//            @Override
//            public void onFailure(org.ros.exception.RemoteException re) {
//                synchronized (SpotSpotPoseLock) {
//                    SpotSpotPoseCondition = true;
//                    SpotSpotPoseSuccess = false;
//                    SpotSpotPoseLock.notify();
//                }
//            }
//        });
//
//        synchronized (SpotSpotPoseLock) {
//            while (!SpotSpotPoseCondition) {
//                try {
//                    SpotSpotPoseLock.wait();
//                } catch (InterruptedException e) {
//                }
//            }
//        }
//        if (SpotSpotPoseSuccess) {
//            edu.tufts.hrilab.diarcros.msg.spot_msgs.SpotPoseResponse.toAde(SpotSpotPoseResponse, response);
//        }
//        return SpotSpotPoseSuccess;
//    }

    public boolean callSpotOpenDoor() {
        SpotOpenDoorCondition = false;
        SpotOpenDoorSuccess = false;
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request = new edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest();
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response = new edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse();
        SpotOpenDoor.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotOpenDoorResponse = mt;
                synchronized (SpotOpenDoorLock) {
                    SpotOpenDoorCondition = true;
                    SpotOpenDoorSuccess = true;
                    SpotOpenDoorLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotOpenDoorLock) {
                    SpotOpenDoorCondition = true;
                    SpotOpenDoorSuccess = false;
                    SpotOpenDoorLock.notify();
                }
            }
        });

        synchronized (SpotOpenDoorLock) {
            while (!SpotOpenDoorCondition) {
                try {
                    SpotOpenDoorLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotOpenDoorSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotOpenDoorResponse, response);
        }
        return SpotOpenDoorSuccess;
    }

    public boolean callSpotUndock() {
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request = new TriggerRequest();
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response = new TriggerResponse();
        SpotUndockCondition = false;
        SpotUndockSuccess = false;
        SpotUndock.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotUndockResponse = mt;
                synchronized (SpotUndockLock) {
                    SpotUndockCondition = true;
                    SpotUndockSuccess = true;
                    SpotUndockLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotUndockLock) {
                    SpotUndockCondition = true;
                    SpotUndockSuccess = false;
                    SpotUndockLock.notify();
                }
            }
        });

        synchronized (SpotUndockLock) {
            while (!SpotUndockCondition) {
                try {
                    SpotUndockLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotUndockSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotUndockResponse, response);
        }
        return SpotUndockSuccess;
    }

    public boolean callSpotRelease(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotReleaseCondition = false;
        SpotReleaseSuccess = false;
        SpotRelease.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotReleaseResponse = mt;
                synchronized (SpotReleaseLock) {
                    SpotReleaseCondition = true;
                    SpotReleaseSuccess = true;
                    SpotReleaseLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotReleaseLock) {
                    SpotReleaseCondition = true;
                    SpotReleaseSuccess = false;
                    SpotReleaseLock.notify();
                }
            }
        });

        synchronized (SpotReleaseLock) {
            while (!SpotReleaseCondition) {
                try {
                    SpotReleaseLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotReleaseSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotReleaseResponse, response);
        }
        return SpotReleaseSuccess;
    }

    public boolean callSpotForceClaim(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotForceClaimCondition = false;
        SpotForceClaimSuccess = false;
        SpotForceClaim.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotForceClaimResponse = mt;
                synchronized (SpotForceClaimLock) {
                    SpotForceClaimCondition = true;
                    SpotForceClaimSuccess = true;
                    SpotForceClaimLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotForceClaimLock) {
                    SpotForceClaimCondition = true;
                    SpotForceClaimSuccess = false;
                    SpotForceClaimLock.notify();
                }
            }
        });

        synchronized (SpotForceClaimLock) {
            while (!SpotForceClaimCondition) {
                try {
                    SpotForceClaimLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotForceClaimSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotForceClaimResponse, response);
        }
        return SpotForceClaimSuccess;
    }

    public boolean callSpotSit() {
        SpotSitCondition = false;
        SpotSitSuccess = false;
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request = new TriggerRequest();
        edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response = new TriggerResponse();
        SpotSit.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotSitResponse = mt;
                synchronized (SpotSitLock) {
                    SpotSitCondition = true;
                    SpotSitSuccess = true;
                    SpotSitLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotSitLock) {
                    SpotSitCondition = true;
                    SpotSitSuccess = false;
                    SpotSitLock.notify();
                }
            }
        });

        synchronized (SpotSitLock) {
            while (!SpotSitCondition) {
                try {
                    SpotSitLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotSitSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotSitResponse, response);
        }
        return SpotSitSuccess;
    }

    public boolean callSpotEstopHard(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotEstopHardCondition = false;
        SpotEstopHardSuccess = false;
        SpotEstopHard.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotEstopHardResponse = mt;
                synchronized (SpotEstopHardLock) {
                    SpotEstopHardCondition = true;
                    SpotEstopHardSuccess = true;
                    SpotEstopHardLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotEstopHardLock) {
                    SpotEstopHardCondition = true;
                    SpotEstopHardSuccess = false;
                    SpotEstopHardLock.notify();
                }
            }
        });

        synchronized (SpotEstopHardLock) {
            while (!SpotEstopHardCondition) {
                try {
                    SpotEstopHardLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotEstopHardSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotEstopHardResponse, response);
        }
        return SpotEstopHardSuccess;
    }

    public boolean callSpotStairMode(edu.tufts.hrilab.diarcros.msg.std_srvs.SetBoolRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.SetBoolResponse response) {
        SpotStairModeCondition = false;
        SpotStairModeSuccess = false;
        SpotStairMode.call(edu.tufts.hrilab.diarcros.msg.std_srvs.SetBoolRequest.toRos(request, node), new ServiceResponseListener<std_srvs.SetBoolResponse>() {

            @Override
            public void onSuccess(std_srvs.SetBoolResponse mt) {
                SpotStairModeResponse = mt;
                synchronized (SpotStairModeLock) {
                    SpotStairModeCondition = true;
                    SpotStairModeSuccess = true;
                    SpotStairModeLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotStairModeLock) {
                    SpotStairModeCondition = true;
                    SpotStairModeSuccess = false;
                    SpotStairModeLock.notify();
                }
            }
        });

        synchronized (SpotStairModeLock) {
            while (!SpotStairModeCondition) {
                try {
                    SpotStairModeLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotStairModeSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.SetBoolResponse.toAde(SpotStairModeResponse, response);
        }
        return SpotStairModeSuccess;
    }

    public boolean callSpotEstopGentle(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse response) {
        SpotEstopGentleCondition = false;
        SpotEstopGentleSuccess = false;
        SpotEstopGentle.call(edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest.toRos(request, node), new ServiceResponseListener<std_srvs.TriggerResponse>() {

            @Override
            public void onSuccess(std_srvs.TriggerResponse mt) {
                SpotEstopGentleResponse = mt;
                synchronized (SpotEstopGentleLock) {
                    SpotEstopGentleCondition = true;
                    SpotEstopGentleSuccess = true;
                    SpotEstopGentleLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotEstopGentleLock) {
                    SpotEstopGentleCondition = true;
                    SpotEstopGentleSuccess = false;
                    SpotEstopGentleLock.notify();
                }
            }
        });

        synchronized (SpotEstopGentleLock) {
            while (!SpotEstopGentleCondition) {
                try {
                    SpotEstopGentleLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotEstopGentleSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse.toAde(SpotEstopGentleResponse, response);
        }
        return SpotEstopGentleSuccess;
    }

    public boolean callSpotDock(edu.tufts.hrilab.diarcros.msg.spot_msgs.DockRequest request, edu.tufts.hrilab.diarcros.msg.spot_msgs.DockResponse response) {
        SpotDockCondition = false;
        SpotDockSuccess = false;
        SpotDock.call(edu.tufts.hrilab.diarcros.msg.spot_msgs.DockRequest.toRos(request, node), new ServiceResponseListener<spot_msgs.DockResponse>() {

            @Override
            public void onSuccess(spot_msgs.DockResponse mt) {
                SpotDockResponse = mt;
                synchronized (SpotDockLock) {
                    SpotDockCondition = true;
                    SpotDockSuccess = true;
                    SpotDockLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SpotDockLock) {
                    SpotDockCondition = true;
                    SpotDockSuccess = false;
                    SpotDockLock.notify();
                }
            }
        });

        synchronized (SpotDockLock) {
            while (!SpotDockCondition) {
                try {
                    SpotDockLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (SpotDockSuccess) {
            edu.tufts.hrilab.diarcros.msg.spot_msgs.DockResponse.toAde(SpotDockResponse, response);
        }
        return SpotDockSuccess;
    }

    // Action(s) Methods
    // public void cancelAllSpotMsgsTrajectoryGoals() {
    //     SpotMsgsTrajectoryClient.cancelAllGoals();
    // }

    // public void cancelSpotMsgsTrajectoryGoal() throws RosException {
    //     SpotMsgsTrajectoryClient.cancelGoal();
    // }

    // public void cancelSpotMsgsTrajectoryGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    //     SpotMsgsTrajectoryClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
    // }

    // public spot_msgs.TrajectoryResult getSpotMsgsTrajectoryResult() throws RosException {
    //     return SpotMsgsTrajectoryClient.getResult();
    // }

    // public SimpleClientGoalState getSpotMsgsTrajectoryState() {
    //     return SpotMsgsTrajectoryClient.getState();
    // }

    // public void sendSpotMsgsTrajectoryGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.TrajectoryGoal goal) throws RosException {
    //     SpotMsgsTrajectoryClient.sendGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.TrajectoryGoal.toRos(goal, node));
    // }

    // public void waitForSpotMsgsTrajectoryResult() throws InterruptedException {
    //     SpotMsgsTrajectoryClient.waitForResult();
    // }

    // public boolean waitForSpotMsgsTrajectoryResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    //     return SpotMsgsTrajectoryClient.waitForResult(timeout, units);
    // }

    // public void waitForSpotMsgsTrajectoryServer() throws InterruptedException {
    //     SpotMsgsTrajectoryClient.waitForServer();
    // }

    // public boolean waitForSpotMsgsTrajectoryServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    //     return SpotMsgsTrajectoryClient.waitForServer(timeout, units);
    // }

    // public void cancelAllSpotMsgsOpenDoorGoals() {
    //     SpotMsgsOpenDoorClient.cancelAllGoals();
    // }

    // public void cancelSpotMsgsOpenDoorGoal() throws RosException {
    //     SpotMsgsOpenDoorClient.cancelGoal();
    // }

    // public void cancelSpotMsgsOpenDoorGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    //     SpotMsgsOpenDoorClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
    // }

    // public spot_msgs.OpenDoorResult getSpotMsgsOpenDoorResult() throws RosException {
    //     return SpotMsgsOpenDoorClient.getResult();
    // }

    // public SimpleClientGoalState getSpotMsgsOpenDoorState() {
    //     return SpotMsgsOpenDoorClient.getState();
    // }

    // public void sendSpotMsgsOpenDoorGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.OpenDoorGoal goal) throws RosException {
    //     SpotMsgsOpenDoorClient.sendGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.OpenDoorGoal.toRos(goal, node));
    // }

    // public void waitForSpotMsgsOpenDoorResult() throws InterruptedException {
    //     SpotMsgsOpenDoorClient.waitForResult();
    // }

    // public boolean waitForSpotMsgsOpenDoorResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    //     return SpotMsgsOpenDoorClient.waitForResult(timeout, units);
    // }

    // public void waitForSpotMsgsOpenDoorServer() throws InterruptedException {
    //     SpotMsgsOpenDoorClient.waitForServer();
    // }

    // public boolean waitForSpotMsgsOpenDoorServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    //     return SpotMsgsOpenDoorClient.waitForServer(timeout, units);
    // }

    // public void cancelAllSpotMsgsNavigateToGoals() {
    //     SpotMsgsNavigateToClient.cancelAllGoals();
    // }

    // public void cancelSpotMsgsNavigateToGoal() throws RosException {
    //     SpotMsgsNavigateToClient.cancelGoal();
    // }

    // public void cancelSpotMsgsNavigateToGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
    //     SpotMsgsNavigateToClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
    // }

    // public spot_msgs.NavigateToResult getSpotMsgsNavigateToResult() throws RosException {
    //     return SpotMsgsNavigateToClient.getResult();
    // }

    // public SimpleClientGoalState getSpotMsgsNavigateToState() {
    //     return SpotMsgsNavigateToClient.getState();
    // }

    // public void sendSpotMsgsNavigateToGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.NavigateToGoal goal) throws RosException {
    //     SpotMsgsNavigateToClient.sendGoal(edu.tufts.hrilab.diarcros.msg.spot_msgs.NavigateToGoal.toRos(goal, node));
    // }

    // public void waitForSpotMsgsNavigateToResult() throws InterruptedException {
    //     SpotMsgsNavigateToClient.waitForResult();
    // }

    // public boolean waitForSpotMsgsNavigateToResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    //     return SpotMsgsNavigateToClient.waitForResult(timeout, units);
    // }

    // public void waitForSpotMsgsNavigateToServer() throws InterruptedException {
    //     SpotMsgsNavigateToClient.waitForServer();
    // }

    // public boolean waitForSpotMsgsNavigateToServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
    //     return SpotMsgsNavigateToClient.waitForServer(timeout, units);
    // }
}
