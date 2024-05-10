/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.common;

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

public class Amcl {
  // ROS connection
  private ConnectedNode node;
  private NodeMain nodeMain;
  private NodeMainExecutor nodeMainExecutor;

  // ROS node ready/wait
  private volatile boolean nodeReady = false;
  private final Lock nodeReadyLock = new ReentrantLock();
  private final Condition nodeReadyCond = nodeReadyLock.newCondition();

  // Local data & Locks
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config AmclParameterUpdates;
  private final Object AmclParameterUpdatesLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
  private final Object RosoutLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseArray Particlecloud;
  private final Object ParticlecloudLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage Tf;
  private final Object TfLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped AmclPose;
  private final Object AmclPoseLock = new Object();
  private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription AmclParameterDescriptions;
  private final Object AmclParameterDescriptionsLock = new Object();

  // Publishers
  private Publisher<rosgraph_msgs.Clock> ClockPublisher;
  private Publisher<geometry_msgs.PoseWithCovarianceStamped> InitialposePublisher;
  private Publisher<sensor_msgs.LaserScan> BaseScanPublisher;
  private Publisher<tf2_msgs.TFMessage> TfStaticPublisher;
  private Publisher<tf2_msgs.TFMessage> TfPublisher;
  // Services
  ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> AmclSetLoggerLevel;
  roscpp.SetLoggerLevelResponse AmclSetLoggerLevelResponse;
  final Object AmclSetLoggerLevelLock = new Object();
  boolean AmclSetLoggerLevelCondition = false;
  boolean AmclSetLoggerLevelSuccess = false;
  ServiceClient<nav_msgs.SetMapRequest, nav_msgs.SetMapResponse> SetMap;
  nav_msgs.SetMapResponse SetMapResponse;
  final Object SetMapLock = new Object();
  boolean SetMapCondition = false;
  boolean SetMapSuccess = false;
  ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> GlobalLocalization;
  std_srvs.EmptyResponse GlobalLocalizationResponse;
  final Object GlobalLocalizationLock = new Object();
  boolean GlobalLocalizationCondition = false;
  boolean GlobalLocalizationSuccess = false;
  ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> AmclGetLoggers;
  roscpp.GetLoggersResponse AmclGetLoggersResponse;
  final Object AmclGetLoggersLock = new Object();
  boolean AmclGetLoggersCondition = false;
  boolean AmclGetLoggersSuccess = false;
  ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> AmclSetParameters;
  dynamic_reconfigure.ReconfigureResponse AmclSetParametersResponse;
  final Object AmclSetParametersLock = new Object();
  boolean AmclSetParametersCondition = false;
  boolean AmclSetParametersSuccess = false;
  ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> RequestNomotionUpdate;
  std_srvs.EmptyResponse RequestNomotionUpdateResponse;
  final Object RequestNomotionUpdateLock = new Object();
  boolean RequestNomotionUpdateCondition = false;
  boolean RequestNomotionUpdateSuccess = false;

  public Amcl() {
    nodeMain = new AbstractNodeMain() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/amcl");
      }

      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        // Subscribers
        Subscriber<dynamic_reconfigure.Config> AmclParameterUpdatesSub = node.newSubscriber("/amcl/parameter_updates", dynamic_reconfigure.Config._TYPE);
        AmclParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.Config msg) {
            synchronized (AmclParameterUpdatesLock) {
              AmclParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
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
        Subscriber<geometry_msgs.PoseArray> ParticlecloudSub = node.newSubscriber("/particlecloud", geometry_msgs.PoseArray._TYPE);
        ParticlecloudSub.addMessageListener(new MessageListener<geometry_msgs.PoseArray>() {
          @Override
          public void onNewMessage(geometry_msgs.PoseArray msg) {
            synchronized (ParticlecloudLock) {
              Particlecloud = edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseArray.toAde(msg);
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
        Subscriber<geometry_msgs.PoseWithCovarianceStamped> AmclPoseSub = node.newSubscriber("/amcl_pose", geometry_msgs.PoseWithCovarianceStamped._TYPE);
        AmclPoseSub.addMessageListener(new MessageListener<geometry_msgs.PoseWithCovarianceStamped>() {
          @Override
          public void onNewMessage(geometry_msgs.PoseWithCovarianceStamped msg) {
            synchronized (AmclPoseLock) {
              AmclPose = edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped.toAde(msg);
            }
          }
        });
        Subscriber<dynamic_reconfigure.ConfigDescription> AmclParameterDescriptionsSub = node.newSubscriber("/amcl/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
        AmclParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
          @Override
          public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
            synchronized (AmclParameterDescriptionsLock) {
              AmclParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
            }
          }
        });
        // Publishers
        ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
        InitialposePublisher = node.newPublisher("/initialpose", geometry_msgs.PoseWithCovarianceStamped._TYPE);
        BaseScanPublisher = node.newPublisher("/base_scan", sensor_msgs.LaserScan._TYPE);
        TfStaticPublisher = node.newPublisher("/tf_static", tf2_msgs.TFMessage._TYPE);
        TfPublisher = node.newPublisher("/tf", tf2_msgs.TFMessage._TYPE);
        //Services
        try {
          AmclSetLoggerLevel = node.newServiceClient("/amcl/set_logger_level", roscpp.SetLoggerLevel._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          SetMap = node.newServiceClient("/set_map", nav_msgs.SetMap._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          GlobalLocalization = node.newServiceClient("/global_localization", std_srvs.Empty._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          AmclGetLoggers = node.newServiceClient("/amcl/get_loggers", roscpp.GetLoggers._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          AmclSetParameters = node.newServiceClient("/amcl/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
        } catch (org.ros.exception.ServiceNotFoundException e) {
          System.err.println("Could not find service! Exception: " + e);
        }
        try {
          RequestNomotionUpdate = node.newServiceClient("/request_nomotion_update", std_srvs.Empty._TYPE);
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

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getAmclParameterUpdates() {
    return AmclParameterUpdates;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
    return Rosout;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseArray getParticlecloud() {
    return Particlecloud;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage getTf() {
    return Tf;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped getAmclPose() {
    return AmclPose;
  }

  public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getAmclParameterDescriptions() {
    return AmclParameterDescriptions;
  }

  public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
    ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
  }

  public void sendInitialpose(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped msg) {
    InitialposePublisher.publish(edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped.toRos(msg, node));
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
  public boolean callAmclSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
    AmclSetLoggerLevelCondition = false;
    AmclSetLoggerLevelSuccess = false;
    AmclSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

              @Override
              public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                AmclSetLoggerLevelResponse = mt;
                synchronized (AmclSetLoggerLevelLock) {
                  AmclSetLoggerLevelCondition = true;
                  AmclSetLoggerLevelSuccess = true;
                  AmclSetLoggerLevelLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (AmclSetLoggerLevelLock) {
                  AmclSetLoggerLevelCondition = true;
                  AmclSetLoggerLevelSuccess = false;
                  AmclSetLoggerLevelLock.notify();
                }
              }
            });

    synchronized (AmclSetLoggerLevelLock) {
      while (!AmclSetLoggerLevelCondition) {
        try {
          AmclSetLoggerLevelLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (AmclSetLoggerLevelSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(AmclSetLoggerLevelResponse, response);
    }
    return AmclSetLoggerLevelSuccess;
  }

  public boolean callSetMap(edu.tufts.hrilab.diarcros.msg.nav_msgs.SetMapRequest request, edu.tufts.hrilab.diarcros.msg.nav_msgs.SetMapResponse response) {
    SetMapCondition = false;
    SetMapSuccess = false;
    SetMap.call(edu.tufts.hrilab.diarcros.msg.nav_msgs.SetMapRequest.toRos(request, node),
            new ServiceResponseListener<nav_msgs.SetMapResponse>() {

              @Override
              public void onSuccess(nav_msgs.SetMapResponse mt) {
                SetMapResponse = mt;
                synchronized (SetMapLock) {
                  SetMapCondition = true;
                  SetMapSuccess = true;
                  SetMapLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (SetMapLock) {
                  SetMapCondition = true;
                  SetMapSuccess = false;
                  SetMapLock.notify();
                }
              }
            });

    synchronized (SetMapLock) {
      while (!SetMapCondition) {
        try {
          SetMapLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (SetMapSuccess) {
      edu.tufts.hrilab.diarcros.msg.nav_msgs.SetMapResponse.toAde(SetMapResponse, response);
    }
    return SetMapSuccess;
  }

  public boolean callGlobalLocalization(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
    GlobalLocalizationCondition = false;
    GlobalLocalizationSuccess = false;
    GlobalLocalization.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node),
            new ServiceResponseListener<std_srvs.EmptyResponse>() {

              @Override
              public void onSuccess(std_srvs.EmptyResponse mt) {
                GlobalLocalizationResponse = mt;
                synchronized (GlobalLocalizationLock) {
                  GlobalLocalizationCondition = true;
                  GlobalLocalizationSuccess = true;
                  GlobalLocalizationLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (GlobalLocalizationLock) {
                  GlobalLocalizationCondition = true;
                  GlobalLocalizationSuccess = false;
                  GlobalLocalizationLock.notify();
                }
              }
            });

    synchronized (GlobalLocalizationLock) {
      while (!GlobalLocalizationCondition) {
        try {
          GlobalLocalizationLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (GlobalLocalizationSuccess) {
      edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(GlobalLocalizationResponse, response);
    }
    return GlobalLocalizationSuccess;
  }

  public boolean callAmclGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
    AmclGetLoggersCondition = false;
    AmclGetLoggersSuccess = false;
    AmclGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node),
            new ServiceResponseListener<roscpp.GetLoggersResponse>() {

              @Override
              public void onSuccess(roscpp.GetLoggersResponse mt) {
                AmclGetLoggersResponse = mt;
                synchronized (AmclGetLoggersLock) {
                  AmclGetLoggersCondition = true;
                  AmclGetLoggersSuccess = true;
                  AmclGetLoggersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (AmclGetLoggersLock) {
                  AmclGetLoggersCondition = true;
                  AmclGetLoggersSuccess = false;
                  AmclGetLoggersLock.notify();
                }
              }
            });

    synchronized (AmclGetLoggersLock) {
      while (!AmclGetLoggersCondition) {
        try {
          AmclGetLoggersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (AmclGetLoggersSuccess) {
      edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(AmclGetLoggersResponse, response);
    }
    return AmclGetLoggersSuccess;
  }

  public boolean callAmclSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
    AmclSetParametersCondition = false;
    AmclSetParametersSuccess = false;
    AmclSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node),
            new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

              @Override
              public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                AmclSetParametersResponse = mt;
                synchronized (AmclSetParametersLock) {
                  AmclSetParametersCondition = true;
                  AmclSetParametersSuccess = true;
                  AmclSetParametersLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (AmclSetParametersLock) {
                  AmclSetParametersCondition = true;
                  AmclSetParametersSuccess = false;
                  AmclSetParametersLock.notify();
                }
              }
            });

    synchronized (AmclSetParametersLock) {
      while (!AmclSetParametersCondition) {
        try {
          AmclSetParametersLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (AmclSetParametersSuccess) {
      edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(AmclSetParametersResponse, response);
    }
    return AmclSetParametersSuccess;
  }

  public boolean callRequestNomotionUpdate(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
    RequestNomotionUpdateCondition = false;
    RequestNomotionUpdateSuccess = false;
    RequestNomotionUpdate.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node),
            new ServiceResponseListener<std_srvs.EmptyResponse>() {

              @Override
              public void onSuccess(std_srvs.EmptyResponse mt) {
                RequestNomotionUpdateResponse = mt;
                synchronized (RequestNomotionUpdateLock) {
                  RequestNomotionUpdateCondition = true;
                  RequestNomotionUpdateSuccess = true;
                  RequestNomotionUpdateLock.notify();
                }
              }

              @Override
              public void onFailure(org.ros.exception.RemoteException re) {
                synchronized (RequestNomotionUpdateLock) {
                  RequestNomotionUpdateCondition = true;
                  RequestNomotionUpdateSuccess = false;
                  RequestNomotionUpdateLock.notify();
                }
              }
            });

    synchronized (RequestNomotionUpdateLock) {
      while (!RequestNomotionUpdateCondition) {
        try {
          RequestNomotionUpdateLock.wait();
        } catch (InterruptedException e) {
        }
      }
    }
    if (RequestNomotionUpdateSuccess) {
      edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(RequestNomotionUpdateResponse, response);
    }
    return RequestNomotionUpdateSuccess;
  }
}
    
