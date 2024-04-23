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

public class ValkyrieHandJoints {
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Local data & Locks
  private edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 RttPeriodOverflow;
  private final Object RttPeriodOverflowLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState HardwareJointCommands;
  private final Object HardwareJointCommandsLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState JointStates;
  private final Object JointStatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
  private final Object RosoutLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.val_hardware_msgs.valAtiSensor ForceTorqueStates;
  private final Object ForceTorqueStatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 RttRate;
  private final Object RttRateLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState HardwareJointStates;
  private final Object HardwareJointStatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.val_hardware_msgs.valImuSensor ImuStates;
  private final Object ImuStatesLock = new Object();

  // Publishers
  private Publisher<std_msgs.Float64MultiArray> LeftHandPositionControllerCommandPublisher;
  private Publisher<std_msgs.Float64MultiArray> RightHandPositionControllerCommandPublisher;
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  // Services
  ServiceClient<controller_manager_msgs.ListControllerTypesRequest, controller_manager_msgs.ListControllerTypesResponse> ControllerManagerListControllerTypes;
  controller_manager_msgs.ListControllerTypesResponse ControllerManagerListControllerTypesResponse;
  final Object ControllerManagerListControllerTypesLock = new Object();
  boolean ControllerManagerListControllerTypesCondition = false;
  boolean ControllerManagerListControllerTypesSuccess = false;
  ServiceClient<controller_manager_msgs.ReloadControllerLibrariesRequest, controller_manager_msgs.ReloadControllerLibrariesResponse> ControllerManagerReloadControllerLibraries;
  controller_manager_msgs.ReloadControllerLibrariesResponse ControllerManagerReloadControllerLibrariesResponse;
  final Object ControllerManagerReloadControllerLibrariesLock = new Object();
  boolean ControllerManagerReloadControllerLibrariesCondition = false;
  boolean ControllerManagerReloadControllerLibrariesSuccess = false;
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> ControllerManagerValk2705153203069839127912SetLoggerLevel;
  roscpp.SetLoggerLevelResponse ControllerManagerValk2705153203069839127912SetLoggerLevelResponse;
  final Object ControllerManagerValk2705153203069839127912SetLoggerLevelLock = new Object();
  boolean ControllerManagerValk2705153203069839127912SetLoggerLevelCondition = false;
  boolean ControllerManagerValk2705153203069839127912SetLoggerLevelSuccess = false;
  ServiceClient<controller_manager_msgs.UnloadControllerRequest, controller_manager_msgs.UnloadControllerResponse> ControllerManagerUnloadController;
  controller_manager_msgs.UnloadControllerResponse ControllerManagerUnloadControllerResponse;
  final Object ControllerManagerUnloadControllerLock = new Object();
  boolean ControllerManagerUnloadControllerCondition = false;
  boolean ControllerManagerUnloadControllerSuccess = false;
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> ControllerManagerValk2705153203069839127912GetLoggers;
  roscpp.GetLoggersResponse ControllerManagerValk2705153203069839127912GetLoggersResponse;
  final Object ControllerManagerValk2705153203069839127912GetLoggersLock = new Object();
  boolean ControllerManagerValk2705153203069839127912GetLoggersCondition = false;
  boolean ControllerManagerValk2705153203069839127912GetLoggersSuccess = false;
  ServiceClient<controller_manager_msgs.ListControllersRequest, controller_manager_msgs.ListControllersResponse> ControllerManagerListControllers;
  controller_manager_msgs.ListControllersResponse ControllerManagerListControllersResponse;
  final Object ControllerManagerListControllersLock = new Object();
  boolean ControllerManagerListControllersCondition = false;
  boolean ControllerManagerListControllersSuccess = false;
  ServiceClient<controller_manager_msgs.SwitchControllerRequest, controller_manager_msgs.SwitchControllerResponse> ControllerManagerSwitchController;
  controller_manager_msgs.SwitchControllerResponse ControllerManagerSwitchControllerResponse;
  final Object ControllerManagerSwitchControllerLock = new Object();
  boolean ControllerManagerSwitchControllerCondition = false;
  boolean ControllerManagerSwitchControllerSuccess = false;
  ServiceClient<controller_manager_msgs.LoadControllerRequest, controller_manager_msgs.LoadControllerResponse> ControllerManagerLoadController;
  controller_manager_msgs.LoadControllerResponse ControllerManagerLoadControllerResponse;
  final Object ControllerManagerLoadControllerLock = new Object();
  boolean ControllerManagerLoadControllerCondition = false;
  boolean ControllerManagerLoadControllerSuccess = false;

  public ValkyrieHandJoints() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/controller_manager_valk_2705_153203069839127912");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<std_msgs.Float64> RttPeriodOverflowSub = node.newSubscriber("/rtt_period_overflow", std_msgs.Float64._TYPE);
        RttPeriodOverflowSub.addMessageListener(new MessageListener<std_msgs.Float64>() {
          @Override
          public void onNewMessage(std_msgs.Float64 msg) {
            synchronized (RttPeriodOverflowLock) {
              RttPeriodOverflow = edu.tufts.hrilab.diarcros.msg.std_msgs.Float64.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.JointState> HardwareJointCommandsSub = node.newSubscriber("/hardware_joint_commands", sensor_msgs.JointState._TYPE);
        HardwareJointCommandsSub.addMessageListener(new MessageListener<sensor_msgs.JointState>() {
          @Override
          public void onNewMessage(sensor_msgs.JointState msg) {
            synchronized (HardwareJointCommandsLock) {
              HardwareJointCommands = edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toAde(msg);
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
        Subscriber<rosgraph_msgs.Log> RosoutSub = node.newSubscriber("/rosout", rosgraph_msgs.Log._TYPE);
        RosoutSub.addMessageListener(new MessageListener<rosgraph_msgs.Log>() {
          @Override
          public void onNewMessage(rosgraph_msgs.Log msg) {
            synchronized (RosoutLock) {
              Rosout = edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log.toAde(msg);
            }
          }
        });
        Subscriber<val_hardware_msgs.valAtiSensor> ForceTorqueStatesSub = node.newSubscriber("/force_torque_states", val_hardware_msgs.valAtiSensor._TYPE);
        ForceTorqueStatesSub.addMessageListener(new MessageListener<val_hardware_msgs.valAtiSensor>() {
          @Override
          public void onNewMessage(val_hardware_msgs.valAtiSensor msg) {
            synchronized (ForceTorqueStatesLock) {
              ForceTorqueStates = edu.tufts.hrilab.diarcros.msg.val_hardware_msgs.valAtiSensor.toAde(msg);
            }
          }
        });
        Subscriber<std_msgs.Float64> RttRateSub = node.newSubscriber("/rtt_rate", std_msgs.Float64._TYPE);
        RttRateSub.addMessageListener(new MessageListener<std_msgs.Float64>() {
          @Override
          public void onNewMessage(std_msgs.Float64 msg) {
            synchronized (RttRateLock) {
              RttRate = edu.tufts.hrilab.diarcros.msg.std_msgs.Float64.toAde(msg);
            }
          }
        });
        Subscriber<sensor_msgs.JointState> HardwareJointStatesSub = node.newSubscriber("/hardware_joint_states", sensor_msgs.JointState._TYPE);
        HardwareJointStatesSub.addMessageListener(new MessageListener<sensor_msgs.JointState>() {
          @Override
          public void onNewMessage(sensor_msgs.JointState msg) {
            synchronized (HardwareJointStatesLock) {
              HardwareJointStates = edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState.toAde(msg);
            }
          }
        });
        Subscriber<val_hardware_msgs.valImuSensor> ImuStatesSub = node.newSubscriber("/imu_states", val_hardware_msgs.valImuSensor._TYPE);
        ImuStatesSub.addMessageListener(new MessageListener<val_hardware_msgs.valImuSensor>() {
          @Override
          public void onNewMessage(val_hardware_msgs.valImuSensor msg) {
            synchronized (ImuStatesLock) {
              ImuStates = edu.tufts.hrilab.diarcros.msg.val_hardware_msgs.valImuSensor.toAde(msg);
            }
          }
        });
        // Publishers
        LeftHandPositionControllerCommandPublisher = node.newPublisher("/left_hand_position_controller/command", std_msgs.Float64MultiArray._TYPE);
        RightHandPositionControllerCommandPublisher = node.newPublisher("/right_hand_position_controller/command", std_msgs.Float64MultiArray._TYPE);
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        //Services
        try {
          ControllerManagerListControllerTypes = node.newServiceClient("/controller_manager/list_controller_types", controller_manager_msgs.ListControllerTypes._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ControllerManagerReloadControllerLibraries = node.newServiceClient("/controller_manager/reload_controller_libraries", controller_manager_msgs.ReloadControllerLibraries._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ControllerManagerValk2705153203069839127912SetLoggerLevel = node.newServiceClient("/controller_manager_valk_2705_153203069839127912/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ControllerManagerUnloadController = node.newServiceClient("/controller_manager/unload_controller", controller_manager_msgs.UnloadController._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ControllerManagerValk2705153203069839127912GetLoggers = node.newServiceClient("/controller_manager_valk_2705_153203069839127912/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ControllerManagerListControllers = node.newServiceClient("/controller_manager/list_controllers", controller_manager_msgs.ListControllers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ControllerManagerSwitchController = node.newServiceClient("/controller_manager/switch_controller", controller_manager_msgs.SwitchController._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          ControllerManagerLoadController = node.newServiceClient("/controller_manager/load_controller", controller_manager_msgs.LoadController._TYPE);
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

  public synchronized edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 getRttPeriodOverflow() {
    return RttPeriodOverflow;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getHardwareJointCommands() {
    return HardwareJointCommands;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getJointStates() {
    return JointStates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
    return Rosout;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.val_hardware_msgs.valAtiSensor getForceTorqueStates() {
    return ForceTorqueStates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.std_msgs.Float64 getRttRate() {
    return RttRate;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState getHardwareJointStates() {
    return HardwareJointStates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.val_hardware_msgs.valImuSensor getImuStates() {
    return ImuStates;
  }

  public void sendLeftHandPositionControllerCommand(edu.tufts.hrilab.diarcros.msg.std_msgs.Float64MultiArray msg) {
    LeftHandPositionControllerCommandPublisher.publish(edu.tufts.hrilab.diarcros.msg.std_msgs.Float64MultiArray.toRos(msg, node));
  }

  public void sendRightHandPositionControllerCommand(edu.tufts.hrilab.diarcros.msg.std_msgs.Float64MultiArray msg) {
    RightHandPositionControllerCommandPublisher.publish(edu.tufts.hrilab.diarcros.msg.std_msgs.Float64MultiArray.toRos(msg, node));
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  // Services
  public boolean callControllerManagerListControllerTypes(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllerTypesRequest request, edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllerTypesResponse response) {
    ControllerManagerListControllerTypesCondition = false;
    ControllerManagerListControllerTypesSuccess = false;
    ControllerManagerListControllerTypes.call(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllerTypesRequest.toRos(request, node),
            new ServiceResponseListener<controller_manager_msgs.ListControllerTypesResponse>() {

              @Override
              public void onSuccess(controller_manager_msgs.ListControllerTypesResponse mt) {
                ControllerManagerListControllerTypesResponse = mt;
                synchronized (ControllerManagerListControllerTypesLock) {
                  ControllerManagerListControllerTypesCondition = true;
                  ControllerManagerListControllerTypesSuccess = true;
                  ControllerManagerListControllerTypesLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerListControllerTypesLock) {
                  ControllerManagerListControllerTypesCondition = true;
                  ControllerManagerListControllerTypesSuccess = false;
                  ControllerManagerListControllerTypesLock.notify();
                }
              }
            });

    synchronized (ControllerManagerListControllerTypesLock) {
      while (!ControllerManagerListControllerTypesCondition) {
        try {
          ControllerManagerListControllerTypesLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerListControllerTypesSuccess) {
      edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllerTypesResponse.toAde(ControllerManagerListControllerTypesResponse, response);
    }
    return ControllerManagerListControllerTypesSuccess;
  }

  public boolean callControllerManagerReloadControllerLibraries(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ReloadControllerLibrariesRequest request, edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ReloadControllerLibrariesResponse response) {
    ControllerManagerReloadControllerLibrariesCondition = false;
    ControllerManagerReloadControllerLibrariesSuccess = false;
    ControllerManagerReloadControllerLibraries.call(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ReloadControllerLibrariesRequest.toRos(request, node),
            new ServiceResponseListener<controller_manager_msgs.ReloadControllerLibrariesResponse>() {

              @Override
              public void onSuccess(controller_manager_msgs.ReloadControllerLibrariesResponse mt) {
                ControllerManagerReloadControllerLibrariesResponse = mt;
                synchronized (ControllerManagerReloadControllerLibrariesLock) {
                  ControllerManagerReloadControllerLibrariesCondition = true;
                  ControllerManagerReloadControllerLibrariesSuccess = true;
                  ControllerManagerReloadControllerLibrariesLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerReloadControllerLibrariesLock) {
                  ControllerManagerReloadControllerLibrariesCondition = true;
                  ControllerManagerReloadControllerLibrariesSuccess = false;
                  ControllerManagerReloadControllerLibrariesLock.notify();
                }
              }
            });

    synchronized (ControllerManagerReloadControllerLibrariesLock) {
      while (!ControllerManagerReloadControllerLibrariesCondition) {
        try {
          ControllerManagerReloadControllerLibrariesLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerReloadControllerLibrariesSuccess) {
      edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ReloadControllerLibrariesResponse.toAde(ControllerManagerReloadControllerLibrariesResponse, response);
    }
    return ControllerManagerReloadControllerLibrariesSuccess;
  }

  public boolean callControllerManagerValk2705153203069839127912SetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
    ControllerManagerValk2705153203069839127912SetLoggerLevelCondition = false;
    ControllerManagerValk2705153203069839127912SetLoggerLevelSuccess = false;
    ControllerManagerValk2705153203069839127912SetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

              @Override
              public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                ControllerManagerValk2705153203069839127912SetLoggerLevelResponse = mt;
                synchronized (ControllerManagerValk2705153203069839127912SetLoggerLevelLock) {
                  ControllerManagerValk2705153203069839127912SetLoggerLevelCondition = true;
                  ControllerManagerValk2705153203069839127912SetLoggerLevelSuccess = true;
                  ControllerManagerValk2705153203069839127912SetLoggerLevelLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerValk2705153203069839127912SetLoggerLevelLock) {
                  ControllerManagerValk2705153203069839127912SetLoggerLevelCondition = true;
                  ControllerManagerValk2705153203069839127912SetLoggerLevelSuccess = false;
                  ControllerManagerValk2705153203069839127912SetLoggerLevelLock.notify();
                }
              }
            });

    synchronized (ControllerManagerValk2705153203069839127912SetLoggerLevelLock) {
      while (!ControllerManagerValk2705153203069839127912SetLoggerLevelCondition) {
        try {
          ControllerManagerValk2705153203069839127912SetLoggerLevelLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerValk2705153203069839127912SetLoggerLevelSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(ControllerManagerValk2705153203069839127912SetLoggerLevelResponse, response);
    }
    return ControllerManagerValk2705153203069839127912SetLoggerLevelSuccess;
  }

  public boolean callControllerManagerUnloadController(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.UnloadControllerRequest request, edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.UnloadControllerResponse response) {
    ControllerManagerUnloadControllerCondition = false;
    ControllerManagerUnloadControllerSuccess = false;
    ControllerManagerUnloadController.call(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.UnloadControllerRequest.toRos(request, node),
            new ServiceResponseListener<controller_manager_msgs.UnloadControllerResponse>() {

              @Override
              public void onSuccess(controller_manager_msgs.UnloadControllerResponse mt) {
                ControllerManagerUnloadControllerResponse = mt;
                synchronized (ControllerManagerUnloadControllerLock) {
                  ControllerManagerUnloadControllerCondition = true;
                  ControllerManagerUnloadControllerSuccess = true;
                  ControllerManagerUnloadControllerLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerUnloadControllerLock) {
                  ControllerManagerUnloadControllerCondition = true;
                  ControllerManagerUnloadControllerSuccess = false;
                  ControllerManagerUnloadControllerLock.notify();
                }
              }
            });

    synchronized (ControllerManagerUnloadControllerLock) {
      while (!ControllerManagerUnloadControllerCondition) {
        try {
          ControllerManagerUnloadControllerLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerUnloadControllerSuccess) {
      edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.UnloadControllerResponse.toAde(ControllerManagerUnloadControllerResponse, response);
    }
    return ControllerManagerUnloadControllerSuccess;
  }

  public boolean callControllerManagerValk2705153203069839127912GetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
    ControllerManagerValk2705153203069839127912GetLoggersCondition = false;
    ControllerManagerValk2705153203069839127912GetLoggersSuccess = false;
    ControllerManagerValk2705153203069839127912GetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.GetLoggersResponse>() {

              @Override
              public void onSuccess(roscpp.GetLoggersResponse mt) {
                ControllerManagerValk2705153203069839127912GetLoggersResponse = mt;
                synchronized (ControllerManagerValk2705153203069839127912GetLoggersLock) {
                  ControllerManagerValk2705153203069839127912GetLoggersCondition = true;
                  ControllerManagerValk2705153203069839127912GetLoggersSuccess = true;
                  ControllerManagerValk2705153203069839127912GetLoggersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerValk2705153203069839127912GetLoggersLock) {
                  ControllerManagerValk2705153203069839127912GetLoggersCondition = true;
                  ControllerManagerValk2705153203069839127912GetLoggersSuccess = false;
                  ControllerManagerValk2705153203069839127912GetLoggersLock.notify();
                }
              }
            });

    synchronized (ControllerManagerValk2705153203069839127912GetLoggersLock) {
      while (!ControllerManagerValk2705153203069839127912GetLoggersCondition) {
        try {
          ControllerManagerValk2705153203069839127912GetLoggersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerValk2705153203069839127912GetLoggersSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(ControllerManagerValk2705153203069839127912GetLoggersResponse, response);
    }
    return ControllerManagerValk2705153203069839127912GetLoggersSuccess;
  }

  public boolean callControllerManagerListControllers(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllersRequest request, edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllersResponse response) {
    ControllerManagerListControllersCondition = false;
    ControllerManagerListControllersSuccess = false;
    ControllerManagerListControllers.call(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllersRequest.toRos(request, node),
            new ServiceResponseListener<controller_manager_msgs.ListControllersResponse>() {

              @Override
              public void onSuccess(controller_manager_msgs.ListControllersResponse mt) {
                ControllerManagerListControllersResponse = mt;
                synchronized (ControllerManagerListControllersLock) {
                  ControllerManagerListControllersCondition = true;
                  ControllerManagerListControllersSuccess = true;
                  ControllerManagerListControllersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerListControllersLock) {
                  ControllerManagerListControllersCondition = true;
                  ControllerManagerListControllersSuccess = false;
                  ControllerManagerListControllersLock.notify();
                }
              }
            });

    synchronized (ControllerManagerListControllersLock) {
      while (!ControllerManagerListControllersCondition) {
        try {
          ControllerManagerListControllersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerListControllersSuccess) {
      edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.ListControllersResponse.toAde(ControllerManagerListControllersResponse, response);
    }
    return ControllerManagerListControllersSuccess;
  }

  public boolean callControllerManagerSwitchController(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.SwitchControllerRequest request, edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.SwitchControllerResponse response) {
    ControllerManagerSwitchControllerCondition = false;
    ControllerManagerSwitchControllerSuccess = false;
    ControllerManagerSwitchController.call(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.SwitchControllerRequest.toRos(request, node),
            new ServiceResponseListener<controller_manager_msgs.SwitchControllerResponse>() {

              @Override
              public void onSuccess(controller_manager_msgs.SwitchControllerResponse mt) {
                ControllerManagerSwitchControllerResponse = mt;
                synchronized (ControllerManagerSwitchControllerLock) {
                  ControllerManagerSwitchControllerCondition = true;
                  ControllerManagerSwitchControllerSuccess = true;
                  ControllerManagerSwitchControllerLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerSwitchControllerLock) {
                  ControllerManagerSwitchControllerCondition = true;
                  ControllerManagerSwitchControllerSuccess = false;
                  ControllerManagerSwitchControllerLock.notify();
                }
              }
            });

    synchronized (ControllerManagerSwitchControllerLock) {
      while (!ControllerManagerSwitchControllerCondition) {
        try {
          ControllerManagerSwitchControllerLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerSwitchControllerSuccess) {
      edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.SwitchControllerResponse.toAde(ControllerManagerSwitchControllerResponse, response);
    }
    return ControllerManagerSwitchControllerSuccess;
  }

  public boolean callControllerManagerLoadController(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.LoadControllerRequest request, edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.LoadControllerResponse response) {
    ControllerManagerLoadControllerCondition = false;
    ControllerManagerLoadControllerSuccess = false;
    ControllerManagerLoadController.call(edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.LoadControllerRequest.toRos(request, node),
            new ServiceResponseListener<controller_manager_msgs.LoadControllerResponse>() {

              @Override
              public void onSuccess(controller_manager_msgs.LoadControllerResponse mt) {
                ControllerManagerLoadControllerResponse = mt;
                synchronized (ControllerManagerLoadControllerLock) {
                  ControllerManagerLoadControllerCondition = true;
                  ControllerManagerLoadControllerSuccess = true;
                  ControllerManagerLoadControllerLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (ControllerManagerLoadControllerLock) {
                  ControllerManagerLoadControllerCondition = true;
                  ControllerManagerLoadControllerSuccess = false;
                  ControllerManagerLoadControllerLock.notify();
                }
              }
            });

    synchronized (ControllerManagerLoadControllerLock) {
      while (!ControllerManagerLoadControllerCondition) {
        try {
          ControllerManagerLoadControllerLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (ControllerManagerLoadControllerSuccess) {
      edu.tufts.hrilab.diarcros.msg.controller_manager_msgs.LoadControllerResponse.toAde(ControllerManagerLoadControllerResponse, response);
    }
    return ControllerManagerLoadControllerSuccess;
  }
}
    
