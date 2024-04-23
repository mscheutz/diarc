/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.ihmc_ros_java_adapter;

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

public class IhmcRosValkyrieController {
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Local data & Locks
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu IhmcRosValkyrieOutputImuTorsoLeftTorsoImu;
  private final Object IhmcRosValkyrieOutputImuTorsoLeftTorsoImuLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage Tf;
  private final Object TfLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.Point2dRosMessage IhmcRosValkyrieOutputCapturabilityCapturePoint;
  private final Object IhmcRosValkyrieOutputCapturabilityCapturePointLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState MultisenseJointStates;
  private final Object MultisenseJointStatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image V1RightHazardCameraCompressed;
  private final Object V1RightHazardCameraCompressedLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo MultisenseLeftImageRectColorCameraInfo;
  private final Object MultisenseLeftImageRectColorCameraInfoLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.std_msgs.Int32 IhmcRosValkyrieOutputBehavior;
  private final Object IhmcRosValkyrieOutputBehaviorLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateChangeStatusRosMessage IhmcRosValkyrieOutputHighLevelStateChange;
  private final Object IhmcRosValkyrieOutputHighLevelStateChangeLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.FootstepStatusRosMessage IhmcRosValkyrieOutputFootstepStatus;
  private final Object IhmcRosValkyrieOutputFootstepStatusLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
  private final Object RosoutLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point32 IhmcRosValkyrieOutputCapturabilityCenterOfMass;
  private final Object IhmcRosValkyrieOutputCapturabilityCenterOfMassLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState IhmcRosValkyrieOutputJointStates;
  private final Object IhmcRosValkyrieOutputJointStatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.WrenchStamped IhmcRosValkyrieOutputFootForceSensorLeft;
  private final Object IhmcRosValkyrieOutputFootForceSensorLeftLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image MultisenseLeftImageRectColor;
  private final Object MultisenseLeftImageRectColorLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.WalkingStatusRosMessage IhmcRosValkyrieOutputWalkingStatus;
  private final Object IhmcRosValkyrieOutputWalkingStatusLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateRosMessage IhmcRosValkyrieOutputHighLevelState;
  private final Object IhmcRosValkyrieOutputHighLevelStateLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image V1LeftHazardCameraCompressed;
  private final Object V1LeftHazardCameraCompressedLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu IhmcRosValkyrieOutputImuPelvisPelvisMiddleImu;
  private final Object IhmcRosValkyrieOutputImuPelvisPelvisMiddleImuLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan MultisenseLidarScan;
  private final Object MultisenseLidarScanLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu IhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensor;
  private final Object IhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensorLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.SupportPolygonRosMessage IhmcRosValkyrieOutputCapturabilityRightFootSupportPolygon;
  private final Object IhmcRosValkyrieOutputCapturabilityRightFootSupportPolygonLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.std_msgs.String IhmcRosValkyrieOutputRobotMotionStatus;
  private final Object IhmcRosValkyrieOutputRobotMotionStatusLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry IhmcRosValkyrieOutputRobotPose;
  private final Object IhmcRosValkyrieOutputRobotPoseLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.WrenchStamped IhmcRosValkyrieOutputFootForceSensorRight;
  private final Object IhmcRosValkyrieOutputFootForceSensorRightLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.SupportPolygonRosMessage IhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygon;
  private final Object IhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygonLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.std_msgs.Bool IhmcRosValkyrieOutputCapturabilityIsInDoubleSupport;
  private final Object IhmcRosValkyrieOutputCapturabilityIsInDoubleSupportLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.ihmc_msgs.Point2dRosMessage IhmcRosValkyrieOutputCapturabilityDesiredCapturePoint;
  private final Object IhmcRosValkyrieOutputCapturabilityDesiredCapturePointLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu IhmcRosValkyrieOutputImuPelvisPelvisRearImu;
  private final Object IhmcRosValkyrieOutputImuPelvisPelvisRearImuLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock Clock;
  private final Object ClockLock = new Object();

  // Publishers
  private Publisher<std_msgs.Empty> IhmcRosValkyrieControlRequestStopPublisher;
  private Publisher<ihmc_msgs.PauseWalkingRosMessage> IhmcRosValkyrieControlPauseWalkingPublisher;
  private Publisher<ihmc_msgs.HighLevelStateRosMessage> IhmcRosValkyrieControlHighLevelStatePublisher;
  private Publisher<ihmc_msgs.GoHomeRosMessage> IhmcRosValkyrieControlGoHomePublisher;
  private Publisher<ihmc_msgs.ArmDesiredAccelerationsRosMessage> IhmcRosValkyrieControlArmDesiredJointAccelerationsPublisher;
  private Publisher<ihmc_msgs.HandTrajectoryRosMessage> IhmcRosValkyrieControlHandTrajectoryPublisher;
  private Publisher<ihmc_msgs.FootTrajectoryRosMessage> IhmcRosValkyrieControlFootTrajectoryPublisher;
  private Publisher<ihmc_msgs.WholeBodyTrajectoryRosMessage> IhmcRosValkyrieControlWholeBodyTrajectoryPublisher;
  private Publisher<ihmc_msgs.EndEffectorLoadBearingRosMessage> IhmcRosValkyrieControlEndEffectorLoadBearingPublisher;
  private Publisher<ihmc_msgs.HandDesiredConfigurationRosMessage> IhmcRosValkyrieControlHandDesiredConfigurationPublisher;
  private Publisher<ihmc_msgs.PelvisTrajectoryRosMessage> IhmcRosValkyrieControlPelvisTrajectoryPublisher;
  private Publisher<ihmc_msgs.StopAllTrajectoryRosMessage> IhmcRosValkyrieControlStopAllTrajectoriesPublisher;
  private Publisher<ihmc_msgs.ChestTrajectoryRosMessage> IhmcRosValkyrieControlChestTrajectoryPublisher;
  private Publisher<ihmc_msgs.FootstepDataListRosMessage> IhmcRosValkyrieControlFootstepListPublisher;
  private Publisher<ihmc_msgs.PelvisHeightTrajectoryRosMessage> IhmcRosValkyrieControlPelvisHeightTrajectoryPublisher;
  private Publisher<ihmc_msgs.NeckDesiredAccelerationsRosMessage> IhmcRosValkyrieControlNeckDesiredAccelerationPublisher;
  private Publisher<ihmc_msgs.ArmTrajectoryRosMessage> IhmcRosValkyrieControlArmTrajectoryPublisher;
  private Publisher<ihmc_msgs.HeadTrajectoryRosMessage> IhmcRosValkyrieControlHeadTrajectoryPublisher;
  private Publisher<ihmc_msgs.NeckTrajectoryRosMessage> IhmcRosValkyrieControlNeckTrajectoryPublisher;
  private Publisher<ihmc_msgs.AbortWalkingRosMessage> IhmcRosValkyrieControlAbortWalkingPublisher;
  private Publisher<ihmc_msgs.PelvisOrientationTrajectoryRosMessage> IhmcRosValkyrieControlPelvisOrientationTrajectoryPublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;

  // Services
  public IhmcRosValkyrieController() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/ihmc_ros/valkyrie/controller");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<sensor_msgs.Imu> IhmcRosValkyrieOutputImuTorsoLeftTorsoImuSub = node.newSubscriber("/ihmc_ros/valkyrie/output/imu/torso_leftTorsoImu", sensor_msgs.Imu._TYPE);
        IhmcRosValkyrieOutputImuTorsoLeftTorsoImuSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
          @Override
          public void onNewMessage(sensor_msgs.Imu msg) {
            synchronized (IhmcRosValkyrieOutputImuTorsoLeftTorsoImuLock) {
              IhmcRosValkyrieOutputImuTorsoLeftTorsoImu = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
            }
          }
        });
        Subscriber<tf2_msgs.TFMessage> TfSub = node.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
        TfSub.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() {
          @Override
          public void onNewMessage(tf2_msgs.TFMessage msg) {
            synchronized (TfLock) {
              Tf = edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.Point2dRosMessage> IhmcRosValkyrieOutputCapturabilityCapturePointSub = node.newSubscriber("/ihmc_ros/valkyrie/output/capturability/capture_point", ihmc_msgs.Point2dRosMessage._TYPE);
        IhmcRosValkyrieOutputCapturabilityCapturePointSub.addMessageListener(new MessageListener<ihmc_msgs.Point2dRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.Point2dRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputCapturabilityCapturePointLock) {
              IhmcRosValkyrieOutputCapturabilityCapturePoint = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.Point2dRosMessage.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.JointState> MultisenseJointStatesSub = node.newSubscriber("/multisense/joint_states", sensor_msgs.JointState._TYPE);
        MultisenseJointStatesSub.addMessageListener(new MessageListener<sensor_msgs.JointState>() {
          @Override
          public void onNewMessage(sensor_msgs.JointState msg) {
            synchronized (MultisenseJointStatesLock) {
              MultisenseJointStates = edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.Image> V1RightHazardCameraCompressedSub = node.newSubscriber("/v1/rightHazardCamera/compressed", sensor_msgs.Image._TYPE);
        V1RightHazardCameraCompressedSub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
          @Override
          public void onNewMessage(sensor_msgs.Image msg) {
            synchronized (V1RightHazardCameraCompressedLock) {
              V1RightHazardCameraCompressed = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.CameraInfo> MultisenseLeftImageRectColorCameraInfoSub = node.newSubscriber("/multisense/left/image_rect_color/camera_info", sensor_msgs.CameraInfo._TYPE);
        MultisenseLeftImageRectColorCameraInfoSub.addMessageListener(new MessageListener<sensor_msgs.CameraInfo>() {
          @Override
          public void onNewMessage(sensor_msgs.CameraInfo msg) {
            synchronized (MultisenseLeftImageRectColorCameraInfoLock) {
              MultisenseLeftImageRectColorCameraInfo = edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo.toAde(msg);
            }
          }
        });
        Subscriber<std_msgs.Int32> IhmcRosValkyrieOutputBehaviorSub = node.newSubscriber("/ihmc_ros/valkyrie/output/behavior", std_msgs.Int32._TYPE);
        IhmcRosValkyrieOutputBehaviorSub.addMessageListener(new MessageListener<std_msgs.Int32>() {
          @Override
          public void onNewMessage(std_msgs.Int32 msg) {
            synchronized (IhmcRosValkyrieOutputBehaviorLock) {
              IhmcRosValkyrieOutputBehavior = edu.tufts.hrilab.diarcros.msg.std_msgs.Int32.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.HighLevelStateChangeStatusRosMessage> IhmcRosValkyrieOutputHighLevelStateChangeSub = node.newSubscriber("/ihmc_ros/valkyrie/output/high_level_state_change", ihmc_msgs.HighLevelStateChangeStatusRosMessage._TYPE);
        IhmcRosValkyrieOutputHighLevelStateChangeSub.addMessageListener(new MessageListener<ihmc_msgs.HighLevelStateChangeStatusRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.HighLevelStateChangeStatusRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputHighLevelStateChangeLock) {
              IhmcRosValkyrieOutputHighLevelStateChange = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateChangeStatusRosMessage.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.FootstepStatusRosMessage> IhmcRosValkyrieOutputFootstepStatusSub = node.newSubscriber("/ihmc_ros/valkyrie/output/footstep_status", ihmc_msgs.FootstepStatusRosMessage._TYPE);
        IhmcRosValkyrieOutputFootstepStatusSub.addMessageListener(new MessageListener<ihmc_msgs.FootstepStatusRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.FootstepStatusRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputFootstepStatusLock) {
              IhmcRosValkyrieOutputFootstepStatus = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.FootstepStatusRosMessage.toAde(msg);
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
        Subscriber<geometry_msgs.Point32> IhmcRosValkyrieOutputCapturabilityCenterOfMassSub = node.newSubscriber("/ihmc_ros/valkyrie/output/capturability/center_of_mass", geometry_msgs.Point32._TYPE);
        IhmcRosValkyrieOutputCapturabilityCenterOfMassSub.addMessageListener(new MessageListener<geometry_msgs.Point32>() {
          @Override
          public void onNewMessage(geometry_msgs.Point32 msg) {
            synchronized (IhmcRosValkyrieOutputCapturabilityCenterOfMassLock) {
              IhmcRosValkyrieOutputCapturabilityCenterOfMass = edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point32.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.JointState> IhmcRosValkyrieOutputJointStatesSub = node.newSubscriber("/ihmc_ros/valkyrie/output/joint_states", sensor_msgs.JointState._TYPE);
        IhmcRosValkyrieOutputJointStatesSub.addMessageListener(new MessageListener<sensor_msgs.JointState>() {
          @Override
          public void onNewMessage(sensor_msgs.JointState msg) {
            synchronized (IhmcRosValkyrieOutputJointStatesLock) {
              IhmcRosValkyrieOutputJointStates = edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toAde(msg);
            }
          }
        });
        Subscriber<geometry_msgs.WrenchStamped> IhmcRosValkyrieOutputFootForceSensorLeftSub = node.newSubscriber("/ihmc_ros/valkyrie/output/foot_force_sensor/left", geometry_msgs.WrenchStamped._TYPE);
        IhmcRosValkyrieOutputFootForceSensorLeftSub.addMessageListener(new MessageListener<geometry_msgs.WrenchStamped>() {
          @Override
          public void onNewMessage(geometry_msgs.WrenchStamped msg) {
            synchronized (IhmcRosValkyrieOutputFootForceSensorLeftLock) {
              IhmcRosValkyrieOutputFootForceSensorLeft = edu.tufts.hrilab.diarcros.msg.geometry_msgs.WrenchStamped.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.Image> MultisenseLeftImageRectColorSub = node.newSubscriber("/multisense/left/image_rect_color", sensor_msgs.Image._TYPE);
        MultisenseLeftImageRectColorSub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
          @Override
          public void onNewMessage(sensor_msgs.Image msg) {
            synchronized (MultisenseLeftImageRectColorLock) {
              MultisenseLeftImageRectColor = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.WalkingStatusRosMessage> IhmcRosValkyrieOutputWalkingStatusSub = node.newSubscriber("/ihmc_ros/valkyrie/output/walking_status", ihmc_msgs.WalkingStatusRosMessage._TYPE);
        IhmcRosValkyrieOutputWalkingStatusSub.addMessageListener(new MessageListener<ihmc_msgs.WalkingStatusRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.WalkingStatusRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputWalkingStatusLock) {
              IhmcRosValkyrieOutputWalkingStatus = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.WalkingStatusRosMessage.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.HighLevelStateRosMessage> IhmcRosValkyrieOutputHighLevelStateSub = node.newSubscriber("/ihmc_ros/valkyrie/output/high_level_state", ihmc_msgs.HighLevelStateRosMessage._TYPE);
        IhmcRosValkyrieOutputHighLevelStateSub.addMessageListener(new MessageListener<ihmc_msgs.HighLevelStateRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.HighLevelStateRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputHighLevelStateLock) {
              IhmcRosValkyrieOutputHighLevelState = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateRosMessage.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.Image> V1LeftHazardCameraCompressedSub = node.newSubscriber("/v1/leftHazardCamera/compressed", sensor_msgs.Image._TYPE);
        V1LeftHazardCameraCompressedSub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
          @Override
          public void onNewMessage(sensor_msgs.Image msg) {
            synchronized (V1LeftHazardCameraCompressedLock) {
              V1LeftHazardCameraCompressed = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.Imu> IhmcRosValkyrieOutputImuPelvisPelvisMiddleImuSub = node.newSubscriber("/ihmc_ros/valkyrie/output/imu/pelvis_pelvisMiddleImu", sensor_msgs.Imu._TYPE);
        IhmcRosValkyrieOutputImuPelvisPelvisMiddleImuSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
          @Override
          public void onNewMessage(sensor_msgs.Imu msg) {
            synchronized (IhmcRosValkyrieOutputImuPelvisPelvisMiddleImuLock) {
              IhmcRosValkyrieOutputImuPelvisPelvisMiddleImu = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.LaserScan> MultisenseLidarScanSub = node.newSubscriber("/multisense/lidar_scan", sensor_msgs.LaserScan._TYPE);
        MultisenseLidarScanSub.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
          @Override
          public void onNewMessage(sensor_msgs.LaserScan msg) {
            synchronized (MultisenseLidarScanLock) {
              MultisenseLidarScan = edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.Imu> IhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensorSub = node.newSubscriber("/ihmc_ros/valkyrie/output/imu/upperNeckPitchLink_head_imu_sensor", sensor_msgs.Imu._TYPE);
        IhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensorSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
          @Override
          public void onNewMessage(sensor_msgs.Imu msg) {
            synchronized (IhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensorLock) {
              IhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensor = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.SupportPolygonRosMessage> IhmcRosValkyrieOutputCapturabilityRightFootSupportPolygonSub = node.newSubscriber("/ihmc_ros/valkyrie/output/capturability/right_foot_support_polygon", ihmc_msgs.SupportPolygonRosMessage._TYPE);
        IhmcRosValkyrieOutputCapturabilityRightFootSupportPolygonSub.addMessageListener(new MessageListener<ihmc_msgs.SupportPolygonRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.SupportPolygonRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputCapturabilityRightFootSupportPolygonLock) {
              IhmcRosValkyrieOutputCapturabilityRightFootSupportPolygon = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.SupportPolygonRosMessage.toAde(msg);
            }
          }
        });
        Subscriber<std_msgs.String> IhmcRosValkyrieOutputRobotMotionStatusSub = node.newSubscriber("/ihmc_ros/valkyrie/output/robot_motion_status", std_msgs.String._TYPE);
        IhmcRosValkyrieOutputRobotMotionStatusSub.addMessageListener(new MessageListener<std_msgs.String>() {
          @Override
          public void onNewMessage(std_msgs.String msg) {
            synchronized (IhmcRosValkyrieOutputRobotMotionStatusLock) {
              IhmcRosValkyrieOutputRobotMotionStatus = edu.tufts.hrilab.diarcros.msg.std_msgs.String.toAde(msg);
            }
          }
        });
        Subscriber<nav_msgs.Odometry> IhmcRosValkyrieOutputRobotPoseSub = node.newSubscriber("/ihmc_ros/valkyrie/output/robot_pose", nav_msgs.Odometry._TYPE);
        IhmcRosValkyrieOutputRobotPoseSub.addMessageListener(new MessageListener<nav_msgs.Odometry>() {
          @Override
          public void onNewMessage(nav_msgs.Odometry msg) {
            synchronized (IhmcRosValkyrieOutputRobotPoseLock) {
              IhmcRosValkyrieOutputRobotPose = edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry.toAde(msg);
            }
          }
        });
        Subscriber<geometry_msgs.WrenchStamped> IhmcRosValkyrieOutputFootForceSensorRightSub = node.newSubscriber("/ihmc_ros/valkyrie/output/foot_force_sensor/right", geometry_msgs.WrenchStamped._TYPE);
        IhmcRosValkyrieOutputFootForceSensorRightSub.addMessageListener(new MessageListener<geometry_msgs.WrenchStamped>() {
          @Override
          public void onNewMessage(geometry_msgs.WrenchStamped msg) {
            synchronized (IhmcRosValkyrieOutputFootForceSensorRightLock) {
              IhmcRosValkyrieOutputFootForceSensorRight = edu.tufts.hrilab.diarcros.msg.geometry_msgs.WrenchStamped.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.SupportPolygonRosMessage> IhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygonSub = node.newSubscriber("/ihmc_ros/valkyrie/output/capturability/left_foot_support_polygon", ihmc_msgs.SupportPolygonRosMessage._TYPE);
        IhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygonSub.addMessageListener(new MessageListener<ihmc_msgs.SupportPolygonRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.SupportPolygonRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygonLock) {
              IhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygon = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.SupportPolygonRosMessage.toAde(msg);
            }
          }
        });
        Subscriber<std_msgs.Bool> IhmcRosValkyrieOutputCapturabilityIsInDoubleSupportSub = node.newSubscriber("/ihmc_ros/valkyrie/output/capturability/is_in_double_support", std_msgs.Bool._TYPE);
        IhmcRosValkyrieOutputCapturabilityIsInDoubleSupportSub.addMessageListener(new MessageListener<std_msgs.Bool>() {
          @Override
          public void onNewMessage(std_msgs.Bool msg) {
            synchronized (IhmcRosValkyrieOutputCapturabilityIsInDoubleSupportLock) {
              IhmcRosValkyrieOutputCapturabilityIsInDoubleSupport = edu.tufts.hrilab.diarcros.msg.std_msgs.Bool.toAde(msg);
            }
          }
        });
        Subscriber<ihmc_msgs.Point2dRosMessage> IhmcRosValkyrieOutputCapturabilityDesiredCapturePointSub = node.newSubscriber("/ihmc_ros/valkyrie/output/capturability/desired_capture_point", ihmc_msgs.Point2dRosMessage._TYPE);
        IhmcRosValkyrieOutputCapturabilityDesiredCapturePointSub.addMessageListener(new MessageListener<ihmc_msgs.Point2dRosMessage>() {
          @Override
          public void onNewMessage(ihmc_msgs.Point2dRosMessage msg) {
            synchronized (IhmcRosValkyrieOutputCapturabilityDesiredCapturePointLock) {
              IhmcRosValkyrieOutputCapturabilityDesiredCapturePoint = edu.tufts.hrilab.diarcros.msg.ihmc_msgs.Point2dRosMessage.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.Imu> IhmcRosValkyrieOutputImuPelvisPelvisRearImuSub = node.newSubscriber("/ihmc_ros/valkyrie/output/imu/pelvis_pelvisRearImu", sensor_msgs.Imu._TYPE);
        IhmcRosValkyrieOutputImuPelvisPelvisRearImuSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
          @Override
          public void onNewMessage(sensor_msgs.Imu msg) {
            synchronized (IhmcRosValkyrieOutputImuPelvisPelvisRearImuLock) {
              IhmcRosValkyrieOutputImuPelvisPelvisRearImu = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
            }
          }
        });
        Subscriber<rosgraph_msgs.Clock> ClockSub = node.newSubscriber("/clock", rosgraph_msgs.Clock._TYPE);
        ClockSub.addMessageListener(new MessageListener<rosgraph_msgs.Clock>() {
          @Override
          public void onNewMessage(rosgraph_msgs.Clock msg) {
            synchronized (ClockLock) {
              Clock = edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toAde(msg);
            }
          }
        });
        // Publishers
        IhmcRosValkyrieControlRequestStopPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/request_stop", std_msgs.Empty._TYPE);
        IhmcRosValkyrieControlPauseWalkingPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/pause_walking", ihmc_msgs.PauseWalkingRosMessage._TYPE);
        IhmcRosValkyrieControlHighLevelStatePublisher = node.newPublisher("/ihmc_ros/valkyrie/control/high_level_state", ihmc_msgs.HighLevelStateRosMessage._TYPE);
        IhmcRosValkyrieControlGoHomePublisher = node.newPublisher("/ihmc_ros/valkyrie/control/go_home", ihmc_msgs.GoHomeRosMessage._TYPE);
        IhmcRosValkyrieControlArmDesiredJointAccelerationsPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/arm_desired_joint_accelerations", ihmc_msgs.ArmDesiredAccelerationsRosMessage._TYPE);
        IhmcRosValkyrieControlHandTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/hand_trajectory", ihmc_msgs.HandTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlFootTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/foot_trajectory", ihmc_msgs.FootTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlWholeBodyTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/whole_body_trajectory", ihmc_msgs.WholeBodyTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlEndEffectorLoadBearingPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/end_effector_load_bearing", ihmc_msgs.EndEffectorLoadBearingRosMessage._TYPE);
        IhmcRosValkyrieControlHandDesiredConfigurationPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/hand_desired_configuration", ihmc_msgs.HandDesiredConfigurationRosMessage._TYPE);
        IhmcRosValkyrieControlPelvisTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/pelvis_trajectory", ihmc_msgs.PelvisTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlStopAllTrajectoriesPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/stop_all_trajectories", ihmc_msgs.StopAllTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlChestTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/chest_trajectory", ihmc_msgs.ChestTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlFootstepListPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/footstep_list", ihmc_msgs.FootstepDataListRosMessage._TYPE);
        IhmcRosValkyrieControlPelvisHeightTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/pelvis_height_trajectory", ihmc_msgs.PelvisHeightTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlNeckDesiredAccelerationPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/neck_desired_acceleration", ihmc_msgs.NeckDesiredAccelerationsRosMessage._TYPE);
        IhmcRosValkyrieControlArmTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/arm_trajectory", ihmc_msgs.ArmTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlHeadTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/head_trajectory", ihmc_msgs.HeadTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlNeckTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/neck_trajectory", ihmc_msgs.NeckTrajectoryRosMessage._TYPE);
        IhmcRosValkyrieControlAbortWalkingPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/abort_walking", ihmc_msgs.AbortWalkingRosMessage._TYPE);
        IhmcRosValkyrieControlPelvisOrientationTrajectoryPublisher = node.newPublisher("/ihmc_ros/valkyrie/control/pelvis_orientation_trajectory", ihmc_msgs.PelvisOrientationTrajectoryRosMessage._TYPE);
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        //Services

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
      String host = InetAddressFactory.newNonLoopback().getHostAddress().toString();
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

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getIhmcRosValkyrieOutputImuTorsoLeftTorsoImu() {
    return IhmcRosValkyrieOutputImuTorsoLeftTorsoImu;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage getTf() {
    return Tf;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.Point2dRosMessage getIhmcRosValkyrieOutputCapturabilityCapturePoint() {
    return IhmcRosValkyrieOutputCapturabilityCapturePoint;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getMultisenseJointStates() {
    return MultisenseJointStates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getV1RightHazardCameraCompressed() {
    return V1RightHazardCameraCompressed;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.CameraInfo getMultisenseLeftImageRectColorCameraInfo() {
    return MultisenseLeftImageRectColorCameraInfo;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.std_msgs.Int32 getIhmcRosValkyrieOutputBehavior() {
    return IhmcRosValkyrieOutputBehavior;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateChangeStatusRosMessage getIhmcRosValkyrieOutputHighLevelStateChange() {
    return IhmcRosValkyrieOutputHighLevelStateChange;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.FootstepStatusRosMessage getIhmcRosValkyrieOutputFootstepStatus() {
    return IhmcRosValkyrieOutputFootstepStatus;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
    return Rosout;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point32 getIhmcRosValkyrieOutputCapturabilityCenterOfMass() {
    return IhmcRosValkyrieOutputCapturabilityCenterOfMass;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getIhmcRosValkyrieOutputJointStates() {
    return IhmcRosValkyrieOutputJointStates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.WrenchStamped getIhmcRosValkyrieOutputFootForceSensorLeft() {
    return IhmcRosValkyrieOutputFootForceSensorLeft;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getMultisenseLeftImageRectColor() {
    return MultisenseLeftImageRectColor;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.WalkingStatusRosMessage getIhmcRosValkyrieOutputWalkingStatus() {
    return IhmcRosValkyrieOutputWalkingStatus;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateRosMessage getIhmcRosValkyrieOutputHighLevelState() {
    return IhmcRosValkyrieOutputHighLevelState;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Image getV1LeftHazardCameraCompressed() {
    return V1LeftHazardCameraCompressed;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getIhmcRosValkyrieOutputImuPelvisPelvisMiddleImu() {
    return IhmcRosValkyrieOutputImuPelvisPelvisMiddleImu;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan getMultisenseLidarScan() {
    return MultisenseLidarScan;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getIhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensor() {
    return IhmcRosValkyrieOutputImuUpperNeckPitchLinkHeadImuSensor;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.SupportPolygonRosMessage getIhmcRosValkyrieOutputCapturabilityRightFootSupportPolygon() {
    return IhmcRosValkyrieOutputCapturabilityRightFootSupportPolygon;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.std_msgs.String getIhmcRosValkyrieOutputRobotMotionStatus() {
    return IhmcRosValkyrieOutputRobotMotionStatus;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry getIhmcRosValkyrieOutputRobotPose() {
    return IhmcRosValkyrieOutputRobotPose;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.WrenchStamped getIhmcRosValkyrieOutputFootForceSensorRight() {
    return IhmcRosValkyrieOutputFootForceSensorRight;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.SupportPolygonRosMessage getIhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygon() {
    return IhmcRosValkyrieOutputCapturabilityLeftFootSupportPolygon;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.std_msgs.Bool getIhmcRosValkyrieOutputCapturabilityIsInDoubleSupport() {
    return IhmcRosValkyrieOutputCapturabilityIsInDoubleSupport;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.ihmc_msgs.Point2dRosMessage getIhmcRosValkyrieOutputCapturabilityDesiredCapturePoint() {
    return IhmcRosValkyrieOutputCapturabilityDesiredCapturePoint;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getIhmcRosValkyrieOutputImuPelvisPelvisRearImu() {
    return IhmcRosValkyrieOutputImuPelvisPelvisRearImu;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock getClock() {
    return Clock;
  }

  public void sendIhmcRosValkyrieControlRequestStop(edu.tufts.hrilab.diarcros.msg.std_msgs.Empty msg) {
    IhmcRosValkyrieControlRequestStopPublisher.publish(edu.tufts.hrilab.diarcros.msg.std_msgs.Empty.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlPauseWalking(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PauseWalkingRosMessage msg) {
    IhmcRosValkyrieControlPauseWalkingPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PauseWalkingRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlHighLevelState(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateRosMessage msg) {
    IhmcRosValkyrieControlHighLevelStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HighLevelStateRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlGoHome(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.GoHomeRosMessage msg) {
    IhmcRosValkyrieControlGoHomePublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.GoHomeRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlArmDesiredJointAccelerations(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.ArmDesiredAccelerationsRosMessage msg) {
    IhmcRosValkyrieControlArmDesiredJointAccelerationsPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.ArmDesiredAccelerationsRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlHandTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HandTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlHandTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HandTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlFootTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.FootTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlFootTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.FootTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlWholeBodyTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.WholeBodyTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlWholeBodyTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.WholeBodyTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlEndEffectorLoadBearing(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.EndEffectorLoadBearingRosMessage msg) {
    IhmcRosValkyrieControlEndEffectorLoadBearingPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.EndEffectorLoadBearingRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlHandDesiredConfiguration(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HandDesiredConfigurationRosMessage msg) {
    IhmcRosValkyrieControlHandDesiredConfigurationPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HandDesiredConfigurationRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlPelvisTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PelvisTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlPelvisTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PelvisTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlStopAllTrajectories(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.StopAllTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlStopAllTrajectoriesPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.StopAllTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlChestTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.ChestTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlChestTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.ChestTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlFootstepList(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.FootstepDataListRosMessage msg) {
    IhmcRosValkyrieControlFootstepListPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.FootstepDataListRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlPelvisHeightTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PelvisHeightTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlPelvisHeightTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PelvisHeightTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlNeckDesiredAcceleration(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.NeckDesiredAccelerationsRosMessage msg) {
    IhmcRosValkyrieControlNeckDesiredAccelerationPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.NeckDesiredAccelerationsRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlArmTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.ArmTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlArmTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.ArmTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlHeadTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HeadTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlHeadTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.HeadTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlNeckTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.NeckTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlNeckTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.NeckTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlAbortWalking(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.AbortWalkingRosMessage msg) {
    IhmcRosValkyrieControlAbortWalkingPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.AbortWalkingRosMessage.toRos(msg, node));
  }

  public void sendIhmcRosValkyrieControlPelvisOrientationTrajectory(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PelvisOrientationTrajectoryRosMessage msg) {
    IhmcRosValkyrieControlPelvisOrientationTrajectoryPublisher.publish(edu.tufts.hrilab.diarcros.msg.ihmc_msgs.PelvisOrientationTrajectoryRosMessage.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }
  // Services
}
    
