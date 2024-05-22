/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.fetch;

import java.net.URI;
import java.net.URISyntaxException;
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

public class RobotDriver {
    //Logging
    protected static Logger log = LoggerFactory.getLogger(RobotDriver.class);

    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // ROS node ready/wait
    private volatile boolean nodeReady = false;
    private final Lock nodeReadyLock = new ReentrantLock();
    private final Condition nodeReadyCond = nodeReadyLock.newCondition();

    // Subscription local data & locks
    private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
    private final Object RosoutLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 Imu2GyroOffsetAge;
    private final Object Imu2GyroOffsetAgeLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu Imu2Imu;
    private final Object Imu2ImuLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry Odom;
    private final Object OdomLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu Imu;
    private final Object ImuLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState JointStates;
    private final Object JointStatesLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped Imu2GyroOffsetSamples;
    private final Object Imu2GyroOffsetSamplesLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Temperature Imu1GyroTemperature;
    private final Object Imu1GyroTemperatureLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu Imu2ImuRaw;
    private final Object Imu2ImuRawLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped Imu1GyroOffsetSamples;
    private final Object Imu1GyroOffsetSamplesLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.power_msgs.BatteryState BatteryState;
    private final Object BatteryStateLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu Imu1ImuRaw;
    private final Object Imu1ImuRawLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.fetch_driver_msgs.RobotState RobotState;
    private final Object RobotStateLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.diagnostic_msgs.DiagnosticArray Diagnostics;
    private final Object DiagnosticsLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped Imu2GyroOffset;
    private final Object Imu2GyroOffsetLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Temperature Imu2GyroTemperature;
    private final Object Imu2GyroTemperatureLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped Imu1GyroOffset;
    private final Object Imu1GyroOffsetLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 Imu1GyroOffsetAge;
    private final Object Imu1GyroOffsetAgeLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu Imu1Imu;
    private final Object Imu1ImuLock = new Object();

    // Publishers
    private Publisher<geometry_msgs.Twist> BaseControllerCommandPublisher;
    private Publisher<geometry_msgs.TwistStamped> ArmControllerCartesianTwistCommandPublisher;
    private Publisher<sensor_msgs.LaserScan> BaseScanPublisher;
    private Publisher<tf2_msgs.TFMessage> TfStaticPublisher;
    private Publisher<tf2_msgs.TFMessage> TfPublisher;
    // Services
    ServiceClient<power_msgs.BreakerCommandRequest, power_msgs.BreakerCommandResponse> ArmBreaker;
    power_msgs.BreakerCommandResponse ArmBreakerResponse;
    final Object ArmBreakerLock = new Object();
    boolean ArmBreakerCondition = false;
    boolean ArmBreakerSuccess = false;
    ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> RobotDriverSetLoggerLevel;
    roscpp.SetLoggerLevelResponse RobotDriverSetLoggerLevelResponse;
    final Object RobotDriverSetLoggerLevelLock = new Object();
    boolean RobotDriverSetLoggerLevelCondition = false;
    boolean RobotDriverSetLoggerLevelSuccess = false;
    ServiceClient<power_msgs.BreakerCommandRequest, power_msgs.BreakerCommandResponse> GripperBreaker;
    power_msgs.BreakerCommandResponse GripperBreakerResponse;
    final Object GripperBreakerLock = new Object();
    boolean GripperBreakerCondition = false;
    boolean GripperBreakerSuccess = false;
    ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> RobotDriverGetLoggers;
    roscpp.GetLoggersResponse RobotDriverGetLoggersResponse;
    final Object RobotDriverGetLoggersLock = new Object();
    boolean RobotDriverGetLoggersCondition = false;
    boolean RobotDriverGetLoggersSuccess = false;
    ServiceClient<power_msgs.BreakerCommandRequest, power_msgs.BreakerCommandResponse> BaseBreaker;
    power_msgs.BreakerCommandResponse BaseBreakerResponse;
    final Object BaseBreakerLock = new Object();
    boolean BaseBreakerCondition = false;
    boolean BaseBreakerSuccess = false;
    // Action clients
    /*
      ControlMsgsPointHead
      control_msgs/PointHead
      control_msgs.PointHeadAction.class
      {:topic &quot;control_msgs/PointHeadAction&quot;, :type &quot;control_msgs.PointHeadAction&quot;}
      ControlMsgsFollowJointTrajectory
      control_msgs/FollowJointTrajectory
      control_msgs.FollowJointTrajectoryAction.class
      {:topic &quot;control_msgs/FollowJointTrajectoryAction&quot;, :type &quot;control_msgs.FollowJointTrajectoryAction&quot;}
      ControlMsgsFollowJointTrajectory
      control_msgs/FollowJointTrajectory
      control_msgs.FollowJointTrajectoryAction.class
      {:topic &quot;control_msgs/FollowJointTrajectoryAction&quot;, :type &quot;control_msgs.FollowJointTrajectoryAction&quot;}
      ControlMsgsFollowJointTrajectory
      control_msgs/FollowJointTrajectory
      control_msgs.FollowJointTrajectoryAction.class
      {:topic &quot;control_msgs/FollowJointTrajectoryAction&quot;, :type &quot;control_msgs.FollowJointTrajectoryAction&quot;}
      FetchDriverMsgsDisableCharging
      fetch_driver_msgs/DisableCharging
      fetch_driver_msgs.DisableChargingAction.class
      {:topic &quot;fetch_driver_msgs/DisableChargingAction&quot;, :type &quot;fetch_driver_msgs.DisableChargingAction&quot;}
      ControlMsgsFollowJointTrajectory
      control_msgs/FollowJointTrajectory
      control_msgs.FollowJointTrajectoryAction.class
      {:topic &quot;control_msgs/FollowJointTrajectoryAction&quot;, :type &quot;control_msgs.FollowJointTrajectoryAction&quot;}
      RobotControllersMsgsQueryControllerStates
      robot_controllers_msgs/QueryControllerStates
      robot_controllers_msgs.QueryControllerStatesAction.class
      {:topic &quot;robot_controllers_msgs/QueryControllerStatesAction&quot;, :type &quot;robot_controllers_msgs.QueryControllerStatesAction&quot;}
    */

    private SimpleActionClient<
            control_msgs.PointHeadActionFeedback,
            control_msgs.PointHeadActionGoal,
            control_msgs.PointHeadActionResult,
            control_msgs.PointHeadFeedback,
            control_msgs.PointHeadGoal,
            control_msgs.PointHeadResult> ControlMsgsPointHeadClient;
        private SimpleActionClient<
            control_msgs.FollowJointTrajectoryActionFeedback,
            control_msgs.FollowJointTrajectoryActionGoal,
            control_msgs.FollowJointTrajectoryActionResult,
            control_msgs.FollowJointTrajectoryFeedback,
            control_msgs.FollowJointTrajectoryGoal,
            control_msgs.FollowJointTrajectoryResult> ControlMsgsFollowJointTrajectoryClient;
        private SimpleActionClient<
            fetch_driver_msgs.DisableChargingActionFeedback,
            fetch_driver_msgs.DisableChargingActionGoal,
            fetch_driver_msgs.DisableChargingActionResult,
            fetch_driver_msgs.DisableChargingFeedback,
            fetch_driver_msgs.DisableChargingGoal,
            fetch_driver_msgs.DisableChargingResult> FetchDriverMsgsDisableChargingClient;
        private SimpleActionClient<
            robot_controllers_msgs.QueryControllerStatesActionFeedback,
            robot_controllers_msgs.QueryControllerStatesActionGoal,
            robot_controllers_msgs.QueryControllerStatesActionResult,
            robot_controllers_msgs.QueryControllerStatesFeedback,
            robot_controllers_msgs.QueryControllerStatesGoal,
            robot_controllers_msgs.QueryControllerStatesResult> RobotControllersMsgsQueryControllerStatesClient;
        

    public RobotDriver() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/robot_driver");
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
                Subscriber<std_msgs.Float64> Imu2GyroOffsetAgeSub = node.newSubscriber("/imu2/gyro_offset_age", std_msgs.Float64._TYPE);
                Imu2GyroOffsetAgeSub.addMessageListener(new MessageListener<std_msgs.Float64>() {
                    @Override
                    public void onNewMessage(std_msgs.Float64 msg) {
                        synchronized (Imu2GyroOffsetAgeLock) {
                            Imu2GyroOffsetAge = edu.tufts.hrilab.diarcros.msg.std_msgs.Float64.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.Imu> Imu2ImuSub = node.newSubscriber("/imu2/imu", sensor_msgs.Imu._TYPE);
                Imu2ImuSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
                    @Override
                    public void onNewMessage(sensor_msgs.Imu msg) {
                        synchronized (Imu2ImuLock) {
                            Imu2Imu = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
                        }
                    }
                });
                Subscriber<nav_msgs.Odometry> OdomSub = node.newSubscriber("/odom", nav_msgs.Odometry._TYPE);
                OdomSub.addMessageListener(new MessageListener<nav_msgs.Odometry>() {
                    @Override
                    public void onNewMessage(nav_msgs.Odometry msg) {
                        synchronized (OdomLock) {
                            Odom = edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.Imu> ImuSub = node.newSubscriber("/imu", sensor_msgs.Imu._TYPE);
                ImuSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
                    @Override
                    public void onNewMessage(sensor_msgs.Imu msg) {
                        synchronized (ImuLock) {
                            Imu = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.JointState> JointStatesSub = node.newSubscriber("/joint_states", sensor_msgs.JointState._TYPE);
                JointStatesSub.addMessageListener(new MessageListener<sensor_msgs.JointState>() {
                    @Override
                    public void onNewMessage(sensor_msgs.JointState msg) {
                        synchronized (JointStatesLock) {
                            JointStates = edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toAde(msg);
                        }
                    }
                });
                Subscriber<geometry_msgs.Vector3Stamped> Imu2GyroOffsetSamplesSub = node.newSubscriber("/imu2/gyro_offset_samples", geometry_msgs.Vector3Stamped._TYPE);
                Imu2GyroOffsetSamplesSub.addMessageListener(new MessageListener<geometry_msgs.Vector3Stamped>() {
                    @Override
                    public void onNewMessage(geometry_msgs.Vector3Stamped msg) {
                        synchronized (Imu2GyroOffsetSamplesLock) {
                            Imu2GyroOffsetSamples = edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.Temperature> Imu1GyroTemperatureSub = node.newSubscriber("/imu1/gyro_temperature", sensor_msgs.Temperature._TYPE);
                Imu1GyroTemperatureSub.addMessageListener(new MessageListener<sensor_msgs.Temperature>() {
                    @Override
                    public void onNewMessage(sensor_msgs.Temperature msg) {
                        synchronized (Imu1GyroTemperatureLock) {
                            Imu1GyroTemperature = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Temperature.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.Imu> Imu2ImuRawSub = node.newSubscriber("/imu2/imu_raw", sensor_msgs.Imu._TYPE);
                Imu2ImuRawSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
                    @Override
                    public void onNewMessage(sensor_msgs.Imu msg) {
                        synchronized (Imu2ImuRawLock) {
                            Imu2ImuRaw = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
                        }
                    }
                });
                Subscriber<geometry_msgs.Vector3Stamped> Imu1GyroOffsetSamplesSub = node.newSubscriber("/imu1/gyro_offset_samples", geometry_msgs.Vector3Stamped._TYPE);
                Imu1GyroOffsetSamplesSub.addMessageListener(new MessageListener<geometry_msgs.Vector3Stamped>() {
                    @Override
                    public void onNewMessage(geometry_msgs.Vector3Stamped msg) {
                        synchronized (Imu1GyroOffsetSamplesLock) {
                            Imu1GyroOffsetSamples = edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped.toAde(msg);
                        }
                    }
                });
                Subscriber<power_msgs.BatteryState> BatteryStateSub = node.newSubscriber("/battery_state", power_msgs.BatteryState._TYPE);
                BatteryStateSub.addMessageListener(new MessageListener<power_msgs.BatteryState>() {
                    @Override
                    public void onNewMessage(power_msgs.BatteryState msg) {
                        synchronized (BatteryStateLock) {
                            BatteryState = edu.tufts.hrilab.diarcros.msg.power_msgs.BatteryState.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.Imu> Imu1ImuRawSub = node.newSubscriber("/imu1/imu_raw", sensor_msgs.Imu._TYPE);
                Imu1ImuRawSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
                    @Override
                    public void onNewMessage(sensor_msgs.Imu msg) {
                        synchronized (Imu1ImuRawLock) {
                            Imu1ImuRaw = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
                        }
                    }
                });
                Subscriber<fetch_driver_msgs.RobotState> RobotStateSub = node.newSubscriber("/robot_state", fetch_driver_msgs.RobotState._TYPE);
                RobotStateSub.addMessageListener(new MessageListener<fetch_driver_msgs.RobotState>() {
                    @Override
                    public void onNewMessage(fetch_driver_msgs.RobotState msg) {
                        synchronized (RobotStateLock) {
                            RobotState = edu.tufts.hrilab.diarcros.msg.fetch_driver_msgs.RobotState.toAde(msg);
                        }
                    }
                });
                Subscriber<diagnostic_msgs.DiagnosticArray> DiagnosticsSub = node.newSubscriber("/diagnostics", diagnostic_msgs.DiagnosticArray._TYPE);
                DiagnosticsSub.addMessageListener(new MessageListener<diagnostic_msgs.DiagnosticArray>() {
                    @Override
                    public void onNewMessage(diagnostic_msgs.DiagnosticArray msg) {
                        synchronized (DiagnosticsLock) {
                            Diagnostics = edu.tufts.hrilab.diarcros.msg.diagnostic_msgs.DiagnosticArray.toAde(msg);
                        }
                    }
                });
                Subscriber<geometry_msgs.Vector3Stamped> Imu2GyroOffsetSub = node.newSubscriber("/imu2/gyro_offset", geometry_msgs.Vector3Stamped._TYPE);
                Imu2GyroOffsetSub.addMessageListener(new MessageListener<geometry_msgs.Vector3Stamped>() {
                    @Override
                    public void onNewMessage(geometry_msgs.Vector3Stamped msg) {
                        synchronized (Imu2GyroOffsetLock) {
                            Imu2GyroOffset = edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.Temperature> Imu2GyroTemperatureSub = node.newSubscriber("/imu2/gyro_temperature", sensor_msgs.Temperature._TYPE);
                Imu2GyroTemperatureSub.addMessageListener(new MessageListener<sensor_msgs.Temperature>() {
                    @Override
                    public void onNewMessage(sensor_msgs.Temperature msg) {
                        synchronized (Imu2GyroTemperatureLock) {
                            Imu2GyroTemperature = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Temperature.toAde(msg);
                        }
                    }
                });
                Subscriber<geometry_msgs.Vector3Stamped> Imu1GyroOffsetSub = node.newSubscriber("/imu1/gyro_offset", geometry_msgs.Vector3Stamped._TYPE);
                Imu1GyroOffsetSub.addMessageListener(new MessageListener<geometry_msgs.Vector3Stamped>() {
                    @Override
                    public void onNewMessage(geometry_msgs.Vector3Stamped msg) {
                        synchronized (Imu1GyroOffsetLock) {
                            Imu1GyroOffset = edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped.toAde(msg);
                        }
                    }
                });
                Subscriber<std_msgs.Float64> Imu1GyroOffsetAgeSub = node.newSubscriber("/imu1/gyro_offset_age", std_msgs.Float64._TYPE);
                Imu1GyroOffsetAgeSub.addMessageListener(new MessageListener<std_msgs.Float64>() {
                    @Override
                    public void onNewMessage(std_msgs.Float64 msg) {
                        synchronized (Imu1GyroOffsetAgeLock) {
                            Imu1GyroOffsetAge = edu.tufts.hrilab.diarcros.msg.std_msgs.Float64.toAde(msg);
                        }
                    }
                });
                Subscriber<sensor_msgs.Imu> Imu1ImuSub = node.newSubscriber("/imu1/imu", sensor_msgs.Imu._TYPE);
                Imu1ImuSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
                    @Override
                    public void onNewMessage(sensor_msgs.Imu msg) {
                        synchronized (Imu1ImuLock) {
                            Imu1Imu = edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu.toAde(msg);
                        }
                    }
                });
                // Publishers
                BaseControllerCommandPublisher = node.newPublisher("/base_controller/command", geometry_msgs.Twist._TYPE);
                ArmControllerCartesianTwistCommandPublisher = node.newPublisher("/arm_controller/cartesian_twist/command", geometry_msgs.TwistStamped._TYPE);
                BaseScanPublisher = node.newPublisher("/base_scan", sensor_msgs.LaserScan._TYPE);
                TfStaticPublisher = node.newPublisher("/tf_static", tf2_msgs.TFMessage._TYPE);
                TfPublisher = node.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);
                //Services
                try {
                    ArmBreaker = node.newServiceClient("/arm_breaker", power_msgs.BreakerCommand._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    RobotDriverSetLoggerLevel = node.newServiceClient("/robot_driver/set_logger_level", roscpp.SetLoggerLevel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GripperBreaker = node.newServiceClient("/gripper_breaker", power_msgs.BreakerCommand._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    RobotDriverGetLoggers = node.newServiceClient("/robot_driver/get_loggers", roscpp.GetLoggers._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    BaseBreaker = node.newServiceClient("/base_breaker", power_msgs.BreakerCommand._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                // Action Client
                try {
                    ControlMsgsPointHeadClient = new SimpleActionClient<>("/robot_driver",
                            new ActionSpec(control_msgs.PointHeadAction.class,
                            "control_msgs/PointHeadAction",
                            "control_msgs/PointHeadActionFeedback",
							"control_msgs/PointHeadActionGoal",
							"control_msgs/PointHeadActionResult",
							"control_msgs/PointHeadFeedback",
							"control_msgs/PointHeadGoal",
							"control_msgs/PointHeadResult"));
                                                    
                    
                } catch (RosException e) {
                    e.printStackTrace();
                }
                while (ControlMsgsPointHeadClient == null)
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                       e.printStackTrace();
                }
                ControlMsgsPointHeadClient.addClientPubSub(node);
                try {
                    ControlMsgsFollowJointTrajectoryClient = new SimpleActionClient<>("/robot_driver",
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
                    ControlMsgsFollowJointTrajectoryClient = new SimpleActionClient<>("/robot_driver",
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
                    ControlMsgsFollowJointTrajectoryClient = new SimpleActionClient<>("/robot_driver",
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
                    FetchDriverMsgsDisableChargingClient = new SimpleActionClient<>("/robot_driver",
                            new ActionSpec(fetch_driver_msgs.DisableChargingAction.class,
                            "fetch_driver_msgs/DisableChargingAction",
                            "fetch_driver_msgs/DisableChargingActionFeedback",
							"fetch_driver_msgs/DisableChargingActionGoal",
							"fetch_driver_msgs/DisableChargingActionResult",
							"fetch_driver_msgs/DisableChargingFeedback",
							"fetch_driver_msgs/DisableChargingGoal",
							"fetch_driver_msgs/DisableChargingResult"));
                                                    
                    
                } catch (RosException e) {
                    e.printStackTrace();
                }
                while (FetchDriverMsgsDisableChargingClient == null)
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                       e.printStackTrace();
                }
                FetchDriverMsgsDisableChargingClient.addClientPubSub(node);
                try {
                    ControlMsgsFollowJointTrajectoryClient = new SimpleActionClient<>("/robot_driver",
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
                    RobotControllersMsgsQueryControllerStatesClient = new SimpleActionClient<>("/robot_driver",
                            new ActionSpec(robot_controllers_msgs.QueryControllerStatesAction.class,
                            "robot_controllers_msgs/QueryControllerStatesAction",
                            "robot_controllers_msgs/QueryControllerStatesActionFeedback",
							"robot_controllers_msgs/QueryControllerStatesActionGoal",
							"robot_controllers_msgs/QueryControllerStatesActionResult",
							"robot_controllers_msgs/QueryControllerStatesFeedback",
							"robot_controllers_msgs/QueryControllerStatesGoal",
							"robot_controllers_msgs/QueryControllerStatesResult"));
                                                    
                    
                } catch (RosException e) {
                    e.printStackTrace();
                }
                while (RobotControllersMsgsQueryControllerStatesClient == null)
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                       e.printStackTrace();
                }
                RobotControllersMsgsQueryControllerStatesClient.addClientPubSub(node);
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
    public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
        return Rosout;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 getImu2GyroOffsetAge() {
        return Imu2GyroOffsetAge;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getImu2Imu() {
        return Imu2Imu;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.nav_msgs.Odometry getOdom() {
        return Odom;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getImu() {
        return Imu;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getJointStates() {
        return JointStates;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped getImu2GyroOffsetSamples() {
        return Imu2GyroOffsetSamples;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Temperature getImu1GyroTemperature() {
        return Imu1GyroTemperature;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getImu2ImuRaw() {
        return Imu2ImuRaw;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped getImu1GyroOffsetSamples() {
        return Imu1GyroOffsetSamples;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.power_msgs.BatteryState getBatteryState() {
        return BatteryState;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getImu1ImuRaw() {
        return Imu1ImuRaw;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.fetch_driver_msgs.RobotState getRobotState() {
        return RobotState;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.diagnostic_msgs.DiagnosticArray getDiagnostics() {
        return Diagnostics;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped getImu2GyroOffset() {
        return Imu2GyroOffset;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Temperature getImu2GyroTemperature() {
        return Imu2GyroTemperature;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3Stamped getImu1GyroOffset() {
        return Imu1GyroOffset;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 getImu1GyroOffsetAge() {
        return Imu1GyroOffsetAge;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.Imu getImu1Imu() {
        return Imu1Imu;
    }
    // Publishers
    public void sendBaseControllerCommand(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist msg) {
        BaseControllerCommandPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist.toRos(msg, node));
    }
    public void sendArmControllerCartesianTwistCommand(edu.tufts.hrilab.diarcros.msg.geometry_msgs.TwistStamped msg) {
        ArmControllerCartesianTwistCommandPublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.TwistStamped.toRos(msg, node));
    }
    public void sendBaseScan(edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan msg) {
        BaseScanPublisher.publish(edu.tufts.hrilab.diarcros.msg.sensor_msgs.LaserScan.toRos(msg, node));
    }
    public void sendTfStatic(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
        TfStaticPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
    }
    public void sendTf(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg) {
        TfPublisher.publish(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage.toRos(msg, node));
    }
    // Services
    public boolean callArmBreaker(edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandRequest request, edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandResponse response) {
        ArmBreakerCondition = false;
        ArmBreakerSuccess = false;
        ArmBreaker.call(edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandRequest.toRos(request, node), 
                new ServiceResponseListener<power_msgs.BreakerCommandResponse>() {

            @Override
            public void onSuccess(power_msgs.BreakerCommandResponse mt) {
                ArmBreakerResponse = mt;
                synchronized(ArmBreakerLock) {
                    ArmBreakerCondition = true;
                    ArmBreakerSuccess = true;
                    ArmBreakerLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(ArmBreakerLock) {
                    ArmBreakerCondition = true;
                    ArmBreakerSuccess = false;
                    ArmBreakerLock.notify();
                }
            }
        });
        
        synchronized(ArmBreakerLock) {
            while(!ArmBreakerCondition) {
                try {
                    ArmBreakerLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (ArmBreakerSuccess) {
            edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandResponse.toAde(ArmBreakerResponse, response);
        }
        return ArmBreakerSuccess;
    }
    public boolean callRobotDriverSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
        RobotDriverSetLoggerLevelCondition = false;
        RobotDriverSetLoggerLevelSuccess = false;
        RobotDriverSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

            @Override
            public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                RobotDriverSetLoggerLevelResponse = mt;
                synchronized(RobotDriverSetLoggerLevelLock) {
                    RobotDriverSetLoggerLevelCondition = true;
                    RobotDriverSetLoggerLevelSuccess = true;
                    RobotDriverSetLoggerLevelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(RobotDriverSetLoggerLevelLock) {
                    RobotDriverSetLoggerLevelCondition = true;
                    RobotDriverSetLoggerLevelSuccess = false;
                    RobotDriverSetLoggerLevelLock.notify();
                }
            }
        });
        
        synchronized(RobotDriverSetLoggerLevelLock) {
            while(!RobotDriverSetLoggerLevelCondition) {
                try {
                    RobotDriverSetLoggerLevelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (RobotDriverSetLoggerLevelSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(RobotDriverSetLoggerLevelResponse, response);
        }
        return RobotDriverSetLoggerLevelSuccess;
    }
    public boolean callGripperBreaker(edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandRequest request, edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandResponse response) {
        GripperBreakerCondition = false;
        GripperBreakerSuccess = false;
        GripperBreaker.call(edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandRequest.toRos(request, node), 
                new ServiceResponseListener<power_msgs.BreakerCommandResponse>() {

            @Override
            public void onSuccess(power_msgs.BreakerCommandResponse mt) {
                GripperBreakerResponse = mt;
                synchronized(GripperBreakerLock) {
                    GripperBreakerCondition = true;
                    GripperBreakerSuccess = true;
                    GripperBreakerLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GripperBreakerLock) {
                    GripperBreakerCondition = true;
                    GripperBreakerSuccess = false;
                    GripperBreakerLock.notify();
                }
            }
        });
        
        synchronized(GripperBreakerLock) {
            while(!GripperBreakerCondition) {
                try {
                    GripperBreakerLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GripperBreakerSuccess) {
            edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandResponse.toAde(GripperBreakerResponse, response);
        }
        return GripperBreakerSuccess;
    }
    public boolean callRobotDriverGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
        RobotDriverGetLoggersCondition = false;
        RobotDriverGetLoggersSuccess = false;
        RobotDriverGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.GetLoggersResponse>() {

            @Override
            public void onSuccess(roscpp.GetLoggersResponse mt) {
                RobotDriverGetLoggersResponse = mt;
                synchronized(RobotDriverGetLoggersLock) {
                    RobotDriverGetLoggersCondition = true;
                    RobotDriverGetLoggersSuccess = true;
                    RobotDriverGetLoggersLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(RobotDriverGetLoggersLock) {
                    RobotDriverGetLoggersCondition = true;
                    RobotDriverGetLoggersSuccess = false;
                    RobotDriverGetLoggersLock.notify();
                }
            }
        });
        
        synchronized(RobotDriverGetLoggersLock) {
            while(!RobotDriverGetLoggersCondition) {
                try {
                    RobotDriverGetLoggersLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (RobotDriverGetLoggersSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(RobotDriverGetLoggersResponse, response);
        }
        return RobotDriverGetLoggersSuccess;
    }
    public boolean callBaseBreaker(edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandRequest request, edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandResponse response) {
        BaseBreakerCondition = false;
        BaseBreakerSuccess = false;
        BaseBreaker.call(edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandRequest.toRos(request, node), 
                new ServiceResponseListener<power_msgs.BreakerCommandResponse>() {

            @Override
            public void onSuccess(power_msgs.BreakerCommandResponse mt) {
                BaseBreakerResponse = mt;
                synchronized(BaseBreakerLock) {
                    BaseBreakerCondition = true;
                    BaseBreakerSuccess = true;
                    BaseBreakerLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(BaseBreakerLock) {
                    BaseBreakerCondition = true;
                    BaseBreakerSuccess = false;
                    BaseBreakerLock.notify();
                }
            }
        });
        
        synchronized(BaseBreakerLock) {
            while(!BaseBreakerCondition) {
                try {
                    BaseBreakerLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (BaseBreakerSuccess) {
            edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandResponse.toAde(BaseBreakerResponse, response);
        }
        return BaseBreakerSuccess;
    }
        
    // Action(s) Methods
    public void cancelAllControlMsgsPointHeadGoals() {
        ControlMsgsPointHeadClient.cancelAllGoals();
    }
        
    public void cancelControlMsgsPointHeadGoal() throws RosException {
        ControlMsgsPointHeadClient.cancelGoal();
    }

    public void cancelControlMsgsPointHeadGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
        ControlMsgsPointHeadClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
    }

    public control_msgs.PointHeadResult getControlMsgsPointHeadResult() throws RosException {
        return ControlMsgsPointHeadClient.getResult();
    }

    public SimpleClientGoalState getControlMsgsPointHeadState() {
        return ControlMsgsPointHeadClient.getState();
    }

    public void sendControlMsgsPointHeadGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.PointHeadGoal goal) throws RosException {
         ControlMsgsPointHeadClient.sendGoal(edu.tufts.hrilab.diarcros.msg.control_msgs.PointHeadGoal.toRos(goal, node));
    }

    public void waitForControlMsgsPointHeadResult() throws InterruptedException {
        ControlMsgsPointHeadClient.waitForResult();
    }

    public boolean waitForControlMsgsPointHeadResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return ControlMsgsPointHeadClient.waitForResult(timeout, units);
    }

    public void waitForControlMsgsPointHeadServer() throws InterruptedException {
        ControlMsgsPointHeadClient.waitForServer();
    }

    public boolean waitForControlMsgsPointHeadServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return ControlMsgsPointHeadClient.waitForServer(timeout, units);
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

    public void cancelAllFetchDriverMsgsDisableChargingGoals() {
        FetchDriverMsgsDisableChargingClient.cancelAllGoals();
    }
        
    public void cancelFetchDriverMsgsDisableChargingGoal() throws RosException {
        FetchDriverMsgsDisableChargingClient.cancelGoal();
    }

    public void cancelFetchDriverMsgsDisableChargingGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
        FetchDriverMsgsDisableChargingClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
    }

    public fetch_driver_msgs.DisableChargingResult getFetchDriverMsgsDisableChargingResult() throws RosException {
        return FetchDriverMsgsDisableChargingClient.getResult();
    }

    public SimpleClientGoalState getFetchDriverMsgsDisableChargingState() {
        return FetchDriverMsgsDisableChargingClient.getState();
    }

    public void sendFetchDriverMsgsDisableChargingGoal(edu.tufts.hrilab.diarcros.msg.fetch_driver_msgs.DisableChargingGoal goal) throws RosException {
         FetchDriverMsgsDisableChargingClient.sendGoal(edu.tufts.hrilab.diarcros.msg.fetch_driver_msgs.DisableChargingGoal.toRos(goal, node));
    }

    public void waitForFetchDriverMsgsDisableChargingResult() throws InterruptedException {
        FetchDriverMsgsDisableChargingClient.waitForResult();
    }

    public boolean waitForFetchDriverMsgsDisableChargingResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return FetchDriverMsgsDisableChargingClient.waitForResult(timeout, units);
    }

    public void waitForFetchDriverMsgsDisableChargingServer() throws InterruptedException {
        FetchDriverMsgsDisableChargingClient.waitForServer();
    }

    public boolean waitForFetchDriverMsgsDisableChargingServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return FetchDriverMsgsDisableChargingClient.waitForServer(timeout, units);
    }

    public void cancelAllRobotControllersMsgsQueryControllerStatesGoals() {
        RobotControllersMsgsQueryControllerStatesClient.cancelAllGoals();
    }
        
    public void cancelRobotControllersMsgsQueryControllerStatesGoal() throws RosException {
        RobotControllersMsgsQueryControllerStatesClient.cancelGoal();
    }

    public void cancelRobotControllersMsgsQueryControllerStatesGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time) {
        RobotControllersMsgsQueryControllerStatesClient.cancelGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time.toRos(time, node));
    }

    public robot_controllers_msgs.QueryControllerStatesResult getRobotControllersMsgsQueryControllerStatesResult() throws RosException {
        return RobotControllersMsgsQueryControllerStatesClient.getResult();
    }

    public SimpleClientGoalState getRobotControllersMsgsQueryControllerStatesState() {
        return RobotControllersMsgsQueryControllerStatesClient.getState();
    }

    public void sendRobotControllersMsgsQueryControllerStatesGoal(edu.tufts.hrilab.diarcros.msg.robot_controllers_msgs.QueryControllerStatesGoal goal) throws RosException {
         RobotControllersMsgsQueryControllerStatesClient.sendGoal(edu.tufts.hrilab.diarcros.msg.robot_controllers_msgs.QueryControllerStatesGoal.toRos(goal, node));
    }

    public void waitForRobotControllersMsgsQueryControllerStatesResult() throws InterruptedException {
        RobotControllersMsgsQueryControllerStatesClient.waitForResult();
    }

    public boolean waitForRobotControllersMsgsQueryControllerStatesResult(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return RobotControllersMsgsQueryControllerStatesClient.waitForResult(timeout, units);
    }

    public void waitForRobotControllersMsgsQueryControllerStatesServer() throws InterruptedException {
        RobotControllersMsgsQueryControllerStatesClient.waitForServer();
    }

    public boolean waitForRobotControllersMsgsQueryControllerStatesServer(long timeout, java.util.concurrent.TimeUnit units) throws InterruptedException {
        return RobotControllersMsgsQueryControllerStatesClient.waitForServer(timeout, units);
    }
}

