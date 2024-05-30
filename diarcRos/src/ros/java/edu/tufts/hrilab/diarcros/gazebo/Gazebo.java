/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.gazebo;

import java.net.URI;
import java.net.URISyntaxException;
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

public class Gazebo {
    // ROS connection
    private ConnectedNode node;
    private NodeMain nodeMain;
    private NodeMainExecutor nodeMainExecutor;

    // Local data & Locks
    private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config GazeboParameterUpdates;
    private final Object GazeboParameterUpdatesLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ModelStates GazeboModelStates;
    private final Object GazeboModelStatesLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log Rosout;
    private final Object RosoutLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.gazebo_msgs.LinkStates GazeboLinkStates;
    private final Object GazeboLinkStatesLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription GazeboParameterDescriptions;
    private final Object GazeboParameterDescriptionsLock = new Object();
    private edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock Clock;
    private final Object ClockLock = new Object();

    // Publishers
    private Publisher<gazebo_msgs.ModelState> GazeboSetModelStatePublisher;
    private Publisher<gazebo_msgs.LinkState> GazeboSetLinkStatePublisher;
    private Publisher<rosgraph_msgs.Clock> ClockPublisher;
    // Services
    ServiceClient<gazebo_msgs.GetModelPropertiesRequest, gazebo_msgs.GetModelPropertiesResponse> GazeboGetModelProperties;
    gazebo_msgs.GetModelPropertiesResponse GazeboGetModelPropertiesResponse;
    final Object GazeboGetModelPropertiesLock = new Object();
    boolean GazeboGetModelPropertiesCondition = false;
    boolean GazeboGetModelPropertiesSuccess = false;
    ServiceClient<gazebo_msgs.ApplyJointEffortRequest, gazebo_msgs.ApplyJointEffortResponse> GazeboApplyJointEffort;
    gazebo_msgs.ApplyJointEffortResponse GazeboApplyJointEffortResponse;
    final Object GazeboApplyJointEffortLock = new Object();
    boolean GazeboApplyJointEffortCondition = false;
    boolean GazeboApplyJointEffortSuccess = false;
    ServiceClient<roscpp.SetLoggerLevelRequest, roscpp.SetLoggerLevelResponse> GazeboSetLoggerLevel;
    roscpp.SetLoggerLevelResponse GazeboSetLoggerLevelResponse;
    final Object GazeboSetLoggerLevelLock = new Object();
    boolean GazeboSetLoggerLevelCondition = false;
    boolean GazeboSetLoggerLevelSuccess = false;
    ServiceClient<gazebo_msgs.GetLinkPropertiesRequest, gazebo_msgs.GetLinkPropertiesResponse> GazeboGetLinkProperties;
    gazebo_msgs.GetLinkPropertiesResponse GazeboGetLinkPropertiesResponse;
    final Object GazeboGetLinkPropertiesLock = new Object();
    boolean GazeboGetLinkPropertiesCondition = false;
    boolean GazeboGetLinkPropertiesSuccess = false;
    ServiceClient<gazebo_msgs.GetPhysicsPropertiesRequest, gazebo_msgs.GetPhysicsPropertiesResponse> GazeboGetPhysicsProperties;
    gazebo_msgs.GetPhysicsPropertiesResponse GazeboGetPhysicsPropertiesResponse;
    final Object GazeboGetPhysicsPropertiesLock = new Object();
    boolean GazeboGetPhysicsPropertiesCondition = false;
    boolean GazeboGetPhysicsPropertiesSuccess = false;
    ServiceClient<gazebo_msgs.DeleteModelRequest, gazebo_msgs.DeleteModelResponse> GazeboDeleteModel;
    gazebo_msgs.DeleteModelResponse GazeboDeleteModelResponse;
    final Object GazeboDeleteModelLock = new Object();
    boolean GazeboDeleteModelCondition = false;
    boolean GazeboDeleteModelSuccess = false;
    ServiceClient<gazebo_msgs.GetLinkStateRequest, gazebo_msgs.GetLinkStateResponse> GazeboGetLinkState;
    gazebo_msgs.GetLinkStateResponse GazeboGetLinkStateResponse;
    final Object GazeboGetLinkStateLock = new Object();
    boolean GazeboGetLinkStateCondition = false;
    boolean GazeboGetLinkStateSuccess = false;
    ServiceClient<gazebo_msgs.GetModelStateRequest, gazebo_msgs.GetModelStateResponse> GazeboGetModelState;
    gazebo_msgs.GetModelStateResponse GazeboGetModelStateResponse;
    final Object GazeboGetModelStateLock = new Object();
    boolean GazeboGetModelStateCondition = false;
    boolean GazeboGetModelStateSuccess = false;
    ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> GazeboResetSimulation;
    std_srvs.EmptyResponse GazeboResetSimulationResponse;
    final Object GazeboResetSimulationLock = new Object();
    boolean GazeboResetSimulationCondition = false;
    boolean GazeboResetSimulationSuccess = false;
    ServiceClient<gazebo_msgs.ApplyBodyWrenchRequest, gazebo_msgs.ApplyBodyWrenchResponse> GazeboApplyBodyWrench;
    gazebo_msgs.ApplyBodyWrenchResponse GazeboApplyBodyWrenchResponse;
    final Object GazeboApplyBodyWrenchLock = new Object();
    boolean GazeboApplyBodyWrenchCondition = false;
    boolean GazeboApplyBodyWrenchSuccess = false;
    ServiceClient<roscpp.GetLoggersRequest, roscpp.GetLoggersResponse> GazeboGetLoggers;
    roscpp.GetLoggersResponse GazeboGetLoggersResponse;
    final Object GazeboGetLoggersLock = new Object();
    boolean GazeboGetLoggersCondition = false;
    boolean GazeboGetLoggersSuccess = false;
    ServiceClient<gazebo_msgs.SpawnModelRequest, gazebo_msgs.SpawnModelResponse> GazeboSpawnSdfModel;
    gazebo_msgs.SpawnModelResponse GazeboSpawnSdfModelResponse;
    final Object GazeboSpawnSdfModelLock = new Object();
    boolean GazeboSpawnSdfModelCondition = false;
    boolean GazeboSpawnSdfModelSuccess = false;
    ServiceClient<gazebo_msgs.SetLinkPropertiesRequest, gazebo_msgs.SetLinkPropertiesResponse> GazeboSetLinkProperties;
    gazebo_msgs.SetLinkPropertiesResponse GazeboSetLinkPropertiesResponse;
    final Object GazeboSetLinkPropertiesLock = new Object();
    boolean GazeboSetLinkPropertiesCondition = false;
    boolean GazeboSetLinkPropertiesSuccess = false;
    ServiceClient<gazebo_msgs.SpawnModelRequest, gazebo_msgs.SpawnModelResponse> GazeboSpawnGazeboModel;
    gazebo_msgs.SpawnModelResponse GazeboSpawnGazeboModelResponse;
    final Object GazeboSpawnGazeboModelLock = new Object();
    boolean GazeboSpawnGazeboModelCondition = false;
    boolean GazeboSpawnGazeboModelSuccess = false;
    ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> GazeboPausePhysics;
    std_srvs.EmptyResponse GazeboPausePhysicsResponse;
    final Object GazeboPausePhysicsLock = new Object();
    boolean GazeboPausePhysicsCondition = false;
    boolean GazeboPausePhysicsSuccess = false;
    ServiceClient<gazebo_msgs.SetModelStateRequest, gazebo_msgs.SetModelStateResponse> GazeboSetModelState;
    gazebo_msgs.SetModelStateResponse GazeboSetModelStateResponse;
    final Object GazeboSetModelStateLock = new Object();
    boolean GazeboSetModelStateCondition = false;
    boolean GazeboSetModelStateSuccess = false;
    ServiceClient<gazebo_msgs.SetJointPropertiesRequest, gazebo_msgs.SetJointPropertiesResponse> GazeboSetJointProperties;
    gazebo_msgs.SetJointPropertiesResponse GazeboSetJointPropertiesResponse;
    final Object GazeboSetJointPropertiesLock = new Object();
    boolean GazeboSetJointPropertiesCondition = false;
    boolean GazeboSetJointPropertiesSuccess = false;
    ServiceClient<gazebo_msgs.BodyRequestRequest, gazebo_msgs.BodyRequestResponse> GazeboClearBodyWrenches;
    gazebo_msgs.BodyRequestResponse GazeboClearBodyWrenchesResponse;
    final Object GazeboClearBodyWrenchesLock = new Object();
    boolean GazeboClearBodyWrenchesCondition = false;
    boolean GazeboClearBodyWrenchesSuccess = false;
    ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> GazeboResetWorld;
    std_srvs.EmptyResponse GazeboResetWorldResponse;
    final Object GazeboResetWorldLock = new Object();
    boolean GazeboResetWorldCondition = false;
    boolean GazeboResetWorldSuccess = false;
    ServiceClient<dynamic_reconfigure.ReconfigureRequest, dynamic_reconfigure.ReconfigureResponse> GazeboSetParameters;
    dynamic_reconfigure.ReconfigureResponse GazeboSetParametersResponse;
    final Object GazeboSetParametersLock = new Object();
    boolean GazeboSetParametersCondition = false;
    boolean GazeboSetParametersSuccess = false;
    ServiceClient<gazebo_msgs.SetModelConfigurationRequest, gazebo_msgs.SetModelConfigurationResponse> GazeboSetModelConfiguration;
    gazebo_msgs.SetModelConfigurationResponse GazeboSetModelConfigurationResponse;
    final Object GazeboSetModelConfigurationLock = new Object();
    boolean GazeboSetModelConfigurationCondition = false;
    boolean GazeboSetModelConfigurationSuccess = false;
    ServiceClient<gazebo_msgs.SetPhysicsPropertiesRequest, gazebo_msgs.SetPhysicsPropertiesResponse> GazeboSetPhysicsProperties;
    gazebo_msgs.SetPhysicsPropertiesResponse GazeboSetPhysicsPropertiesResponse;
    final Object GazeboSetPhysicsPropertiesLock = new Object();
    boolean GazeboSetPhysicsPropertiesCondition = false;
    boolean GazeboSetPhysicsPropertiesSuccess = false;
    ServiceClient<gazebo_msgs.GetWorldPropertiesRequest, gazebo_msgs.GetWorldPropertiesResponse> GazeboGetWorldProperties;
    gazebo_msgs.GetWorldPropertiesResponse GazeboGetWorldPropertiesResponse;
    final Object GazeboGetWorldPropertiesLock = new Object();
    boolean GazeboGetWorldPropertiesCondition = false;
    boolean GazeboGetWorldPropertiesSuccess = false;
    ServiceClient<gazebo_msgs.SetLinkStateRequest, gazebo_msgs.SetLinkStateResponse> GazeboSetLinkState;
    gazebo_msgs.SetLinkStateResponse GazeboSetLinkStateResponse;
    final Object GazeboSetLinkStateLock = new Object();
    boolean GazeboSetLinkStateCondition = false;
    boolean GazeboSetLinkStateSuccess = false;
    ServiceClient<std_srvs.EmptyRequest, std_srvs.EmptyResponse> GazeboUnpausePhysics;
    std_srvs.EmptyResponse GazeboUnpausePhysicsResponse;
    final Object GazeboUnpausePhysicsLock = new Object();
    boolean GazeboUnpausePhysicsCondition = false;
    boolean GazeboUnpausePhysicsSuccess = false;
    ServiceClient<gazebo_msgs.SpawnModelRequest, gazebo_msgs.SpawnModelResponse> GazeboSpawnUrdfModel;
    gazebo_msgs.SpawnModelResponse GazeboSpawnUrdfModelResponse;
    final Object GazeboSpawnUrdfModelLock = new Object();
    boolean GazeboSpawnUrdfModelCondition = false;
    boolean GazeboSpawnUrdfModelSuccess = false;
    ServiceClient<gazebo_msgs.JointRequestRequest, gazebo_msgs.JointRequestResponse> GazeboClearJointForces;
    gazebo_msgs.JointRequestResponse GazeboClearJointForcesResponse;
    final Object GazeboClearJointForcesLock = new Object();
    boolean GazeboClearJointForcesCondition = false;
    boolean GazeboClearJointForcesSuccess = false;
    ServiceClient<gazebo_msgs.GetJointPropertiesRequest, gazebo_msgs.GetJointPropertiesResponse> GazeboGetJointProperties;
    gazebo_msgs.GetJointPropertiesResponse GazeboGetJointPropertiesResponse;
    final Object GazeboGetJointPropertiesLock = new Object();
    boolean GazeboGetJointPropertiesCondition = false;
    boolean GazeboGetJointPropertiesSuccess = false;
    public Gazebo() {
        nodeMain = new AbstractNodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ade/gazebo");
            }
                
            @Override
            public void onStart(ConnectedNode connectedNode) {
                node = connectedNode;
                // Subscribers
                Subscriber<dynamic_reconfigure.Config> GazeboParameterUpdatesSub = node.newSubscriber("/gazebo/parameter_updates", dynamic_reconfigure.Config._TYPE);
                GazeboParameterUpdatesSub.addMessageListener(new MessageListener<dynamic_reconfigure.Config>() {
                    @Override
                    public void onNewMessage(dynamic_reconfigure.Config msg) {
                        synchronized (GazeboParameterUpdatesLock) {
                            GazeboParameterUpdates = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config.toAde(msg);
                        }
                    }
                });
                Subscriber<gazebo_msgs.ModelStates> GazeboModelStatesSub = node.newSubscriber("/gazebo/model_states", gazebo_msgs.ModelStates._TYPE);
                GazeboModelStatesSub.addMessageListener(new MessageListener<gazebo_msgs.ModelStates>() {
                    @Override
                    public void onNewMessage(gazebo_msgs.ModelStates msg) {
                        synchronized (GazeboModelStatesLock) {
                            GazeboModelStates = edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ModelStates.toAde(msg);
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
                Subscriber<gazebo_msgs.LinkStates> GazeboLinkStatesSub = node.newSubscriber("/gazebo/link_states", gazebo_msgs.LinkStates._TYPE);
                GazeboLinkStatesSub.addMessageListener(new MessageListener<gazebo_msgs.LinkStates>() {
                    @Override
                    public void onNewMessage(gazebo_msgs.LinkStates msg) {
                        synchronized (GazeboLinkStatesLock) {
                            GazeboLinkStates = edu.tufts.hrilab.diarcros.msg.gazebo_msgs.LinkStates.toAde(msg);
                        }
                    }
                });
                Subscriber<dynamic_reconfigure.ConfigDescription> GazeboParameterDescriptionsSub = node.newSubscriber("/gazebo/parameter_descriptions", dynamic_reconfigure.ConfigDescription._TYPE);
                GazeboParameterDescriptionsSub.addMessageListener(new MessageListener<dynamic_reconfigure.ConfigDescription>() {
                    @Override
                    public void onNewMessage(dynamic_reconfigure.ConfigDescription msg) {
                        synchronized (GazeboParameterDescriptionsLock) {
                            GazeboParameterDescriptions = edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription.toAde(msg);
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
                GazeboSetModelStatePublisher = node.newPublisher("/gazebo/set_model_state", gazebo_msgs.ModelState._TYPE);
                GazeboSetLinkStatePublisher = node.newPublisher("/gazebo/set_link_state", gazebo_msgs.LinkState._TYPE);
                ClockPublisher = node.newPublisher("/clock", rosgraph_msgs.Clock._TYPE);
                //Services
                try {
                    GazeboGetModelProperties = node.newServiceClient("/gazebo/get_model_properties", gazebo_msgs.GetModelProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboApplyJointEffort = node.newServiceClient("/gazebo/apply_joint_effort", gazebo_msgs.ApplyJointEffort._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetLoggerLevel = node.newServiceClient("/gazebo/set_logger_level", roscpp.SetLoggerLevel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboGetLinkProperties = node.newServiceClient("/gazebo/get_link_properties", gazebo_msgs.GetLinkProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboGetPhysicsProperties = node.newServiceClient("/gazebo/get_physics_properties", gazebo_msgs.GetPhysicsProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboDeleteModel = node.newServiceClient("/gazebo/delete_model", gazebo_msgs.DeleteModel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboGetLinkState = node.newServiceClient("/gazebo/get_link_state", gazebo_msgs.GetLinkState._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboGetModelState = node.newServiceClient("/gazebo/get_model_state", gazebo_msgs.GetModelState._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboResetSimulation = node.newServiceClient("/gazebo/reset_simulation", std_srvs.Empty._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboApplyBodyWrench = node.newServiceClient("/gazebo/apply_body_wrench", gazebo_msgs.ApplyBodyWrench._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboGetLoggers = node.newServiceClient("/gazebo/get_loggers", roscpp.GetLoggers._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSpawnSdfModel = node.newServiceClient("/gazebo/spawn_sdf_model", gazebo_msgs.SpawnModel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetLinkProperties = node.newServiceClient("/gazebo/set_link_properties", gazebo_msgs.SetLinkProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSpawnGazeboModel = node.newServiceClient("/gazebo/spawn_gazebo_model", gazebo_msgs.SpawnModel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboPausePhysics = node.newServiceClient("/gazebo/pause_physics", std_srvs.Empty._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetModelState = node.newServiceClient("/gazebo/set_model_state", gazebo_msgs.SetModelState._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetJointProperties = node.newServiceClient("/gazebo/set_joint_properties", gazebo_msgs.SetJointProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboClearBodyWrenches = node.newServiceClient("/gazebo/clear_body_wrenches", gazebo_msgs.BodyRequest._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboResetWorld = node.newServiceClient("/gazebo/reset_world", std_srvs.Empty._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetParameters = node.newServiceClient("/gazebo/set_parameters", dynamic_reconfigure.Reconfigure._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetModelConfiguration = node.newServiceClient("/gazebo/set_model_configuration", gazebo_msgs.SetModelConfiguration._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetPhysicsProperties = node.newServiceClient("/gazebo/set_physics_properties", gazebo_msgs.SetPhysicsProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboGetWorldProperties = node.newServiceClient("/gazebo/get_world_properties", gazebo_msgs.GetWorldProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSetLinkState = node.newServiceClient("/gazebo/set_link_state", gazebo_msgs.SetLinkState._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboUnpausePhysics = node.newServiceClient("/gazebo/unpause_physics", std_srvs.Empty._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboSpawnUrdfModel = node.newServiceClient("/gazebo/spawn_urdf_model", gazebo_msgs.SpawnModel._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboClearJointForces = node.newServiceClient("/gazebo/clear_joint_forces", gazebo_msgs.JointRequest._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
                }
                try {
                    GazeboGetJointProperties = node.newServiceClient("/gazebo/get_joint_properties", gazebo_msgs.GetJointProperties._TYPE);
                } catch (org.ros.exception.ServiceNotFoundException e) {
                    System.err.println("Could not find service! Exception: " + e);
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

    
    public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getGazeboParameterUpdates() {
        return GazeboParameterUpdates;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ModelStates getGazeboModelStates() {
        return GazeboModelStates;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout() {
        return Rosout;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.gazebo_msgs.LinkStates getGazeboLinkStates() {
        return GazeboLinkStates;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getGazeboParameterDescriptions() {
        return GazeboParameterDescriptions;
    }
    public synchronized edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock getClock() {
        return Clock;
    }
    public void sendGazeboSetModelState(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ModelState msg) {
        GazeboSetModelStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ModelState.toRos(msg, node));
    }
    public void sendGazeboSetLinkState(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.LinkState msg) {
        GazeboSetLinkStatePublisher.publish(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.LinkState.toRos(msg, node));
    }
    public void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg) {
        ClockPublisher.publish(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock.toRos(msg, node));
    }
    // Services
    public boolean callGazeboGetModelProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelPropertiesResponse response) {
        GazeboGetModelPropertiesCondition = false;
        GazeboGetModelPropertiesSuccess = false;
        GazeboGetModelProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.GetModelPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.GetModelPropertiesResponse mt) {
                GazeboGetModelPropertiesResponse = mt;
                synchronized(GazeboGetModelPropertiesLock) {
                    GazeboGetModelPropertiesCondition = true;
                    GazeboGetModelPropertiesSuccess = true;
                    GazeboGetModelPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetModelPropertiesLock) {
                    GazeboGetModelPropertiesCondition = true;
                    GazeboGetModelPropertiesSuccess = false;
                    GazeboGetModelPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetModelPropertiesLock) {
            while(!GazeboGetModelPropertiesCondition) {
                try {
                    GazeboGetModelPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetModelPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelPropertiesResponse.toAde(GazeboGetModelPropertiesResponse, response);
        }
        return GazeboGetModelPropertiesSuccess;
    }
    public boolean callGazeboApplyJointEffort(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyJointEffortRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyJointEffortResponse response) {
        GazeboApplyJointEffortCondition = false;
        GazeboApplyJointEffortSuccess = false;
        GazeboApplyJointEffort.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyJointEffortRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.ApplyJointEffortResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.ApplyJointEffortResponse mt) {
                GazeboApplyJointEffortResponse = mt;
                synchronized(GazeboApplyJointEffortLock) {
                    GazeboApplyJointEffortCondition = true;
                    GazeboApplyJointEffortSuccess = true;
                    GazeboApplyJointEffortLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboApplyJointEffortLock) {
                    GazeboApplyJointEffortCondition = true;
                    GazeboApplyJointEffortSuccess = false;
                    GazeboApplyJointEffortLock.notify();
                }
            }
        });
        
        synchronized(GazeboApplyJointEffortLock) {
            while(!GazeboApplyJointEffortCondition) {
                try {
                    GazeboApplyJointEffortLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboApplyJointEffortSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyJointEffortResponse.toAde(GazeboApplyJointEffortResponse, response);
        }
        return GazeboApplyJointEffortSuccess;
    }
    public boolean callGazeboSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response) {
        GazeboSetLoggerLevelCondition = false;
        GazeboSetLoggerLevelSuccess = false;
        GazeboSetLoggerLevel.call(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.SetLoggerLevelResponse>() {

            @Override
            public void onSuccess(roscpp.SetLoggerLevelResponse mt) {
                GazeboSetLoggerLevelResponse = mt;
                synchronized(GazeboSetLoggerLevelLock) {
                    GazeboSetLoggerLevelCondition = true;
                    GazeboSetLoggerLevelSuccess = true;
                    GazeboSetLoggerLevelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetLoggerLevelLock) {
                    GazeboSetLoggerLevelCondition = true;
                    GazeboSetLoggerLevelSuccess = false;
                    GazeboSetLoggerLevelLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetLoggerLevelLock) {
            while(!GazeboSetLoggerLevelCondition) {
                try {
                    GazeboSetLoggerLevelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetLoggerLevelSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse.toAde(GazeboSetLoggerLevelResponse, response);
        }
        return GazeboSetLoggerLevelSuccess;
    }
    public boolean callGazeboGetLinkProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkPropertiesResponse response) {
        GazeboGetLinkPropertiesCondition = false;
        GazeboGetLinkPropertiesSuccess = false;
        GazeboGetLinkProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.GetLinkPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.GetLinkPropertiesResponse mt) {
                GazeboGetLinkPropertiesResponse = mt;
                synchronized(GazeboGetLinkPropertiesLock) {
                    GazeboGetLinkPropertiesCondition = true;
                    GazeboGetLinkPropertiesSuccess = true;
                    GazeboGetLinkPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetLinkPropertiesLock) {
                    GazeboGetLinkPropertiesCondition = true;
                    GazeboGetLinkPropertiesSuccess = false;
                    GazeboGetLinkPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetLinkPropertiesLock) {
            while(!GazeboGetLinkPropertiesCondition) {
                try {
                    GazeboGetLinkPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetLinkPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkPropertiesResponse.toAde(GazeboGetLinkPropertiesResponse, response);
        }
        return GazeboGetLinkPropertiesSuccess;
    }
    public boolean callGazeboGetPhysicsProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetPhysicsPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetPhysicsPropertiesResponse response) {
        GazeboGetPhysicsPropertiesCondition = false;
        GazeboGetPhysicsPropertiesSuccess = false;
        GazeboGetPhysicsProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetPhysicsPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.GetPhysicsPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.GetPhysicsPropertiesResponse mt) {
                GazeboGetPhysicsPropertiesResponse = mt;
                synchronized(GazeboGetPhysicsPropertiesLock) {
                    GazeboGetPhysicsPropertiesCondition = true;
                    GazeboGetPhysicsPropertiesSuccess = true;
                    GazeboGetPhysicsPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetPhysicsPropertiesLock) {
                    GazeboGetPhysicsPropertiesCondition = true;
                    GazeboGetPhysicsPropertiesSuccess = false;
                    GazeboGetPhysicsPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetPhysicsPropertiesLock) {
            while(!GazeboGetPhysicsPropertiesCondition) {
                try {
                    GazeboGetPhysicsPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetPhysicsPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetPhysicsPropertiesResponse.toAde(GazeboGetPhysicsPropertiesResponse, response);
        }
        return GazeboGetPhysicsPropertiesSuccess;
    }
    public boolean callGazeboDeleteModel(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.DeleteModelRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.DeleteModelResponse response) {
        GazeboDeleteModelCondition = false;
        GazeboDeleteModelSuccess = false;
        GazeboDeleteModel.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.DeleteModelRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.DeleteModelResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.DeleteModelResponse mt) {
                GazeboDeleteModelResponse = mt;
                synchronized(GazeboDeleteModelLock) {
                    GazeboDeleteModelCondition = true;
                    GazeboDeleteModelSuccess = true;
                    GazeboDeleteModelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboDeleteModelLock) {
                    GazeboDeleteModelCondition = true;
                    GazeboDeleteModelSuccess = false;
                    GazeboDeleteModelLock.notify();
                }
            }
        });
        
        synchronized(GazeboDeleteModelLock) {
            while(!GazeboDeleteModelCondition) {
                try {
                    GazeboDeleteModelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboDeleteModelSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.DeleteModelResponse.toAde(GazeboDeleteModelResponse, response);
        }
        return GazeboDeleteModelSuccess;
    }
    public boolean callGazeboGetLinkState(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkStateRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkStateResponse response) {
        GazeboGetLinkStateCondition = false;
        GazeboGetLinkStateSuccess = false;
        GazeboGetLinkState.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkStateRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.GetLinkStateResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.GetLinkStateResponse mt) {
                GazeboGetLinkStateResponse = mt;
                synchronized(GazeboGetLinkStateLock) {
                    GazeboGetLinkStateCondition = true;
                    GazeboGetLinkStateSuccess = true;
                    GazeboGetLinkStateLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetLinkStateLock) {
                    GazeboGetLinkStateCondition = true;
                    GazeboGetLinkStateSuccess = false;
                    GazeboGetLinkStateLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetLinkStateLock) {
            while(!GazeboGetLinkStateCondition) {
                try {
                    GazeboGetLinkStateLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetLinkStateSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetLinkStateResponse.toAde(GazeboGetLinkStateResponse, response);
        }
        return GazeboGetLinkStateSuccess;
    }
    public boolean callGazeboGetModelState(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelStateRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelStateResponse response) {
        GazeboGetModelStateCondition = false;
        GazeboGetModelStateSuccess = false;
        GazeboGetModelState.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelStateRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.GetModelStateResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.GetModelStateResponse mt) {
                GazeboGetModelStateResponse = mt;
                synchronized(GazeboGetModelStateLock) {
                    GazeboGetModelStateCondition = true;
                    GazeboGetModelStateSuccess = true;
                    GazeboGetModelStateLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetModelStateLock) {
                    GazeboGetModelStateCondition = true;
                    GazeboGetModelStateSuccess = false;
                    GazeboGetModelStateLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetModelStateLock) {
            while(!GazeboGetModelStateCondition) {
                try {
                    GazeboGetModelStateLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetModelStateSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetModelStateResponse.toAde(GazeboGetModelStateResponse, response);
        }
        return GazeboGetModelStateSuccess;
    }
    public boolean callGazeboResetSimulation(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
        GazeboResetSimulationCondition = false;
        GazeboResetSimulationSuccess = false;
        GazeboResetSimulation.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node), 
                new ServiceResponseListener<std_srvs.EmptyResponse>() {

            @Override
            public void onSuccess(std_srvs.EmptyResponse mt) {
                GazeboResetSimulationResponse = mt;
                synchronized(GazeboResetSimulationLock) {
                    GazeboResetSimulationCondition = true;
                    GazeboResetSimulationSuccess = true;
                    GazeboResetSimulationLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboResetSimulationLock) {
                    GazeboResetSimulationCondition = true;
                    GazeboResetSimulationSuccess = false;
                    GazeboResetSimulationLock.notify();
                }
            }
        });
        
        synchronized(GazeboResetSimulationLock) {
            while(!GazeboResetSimulationCondition) {
                try {
                    GazeboResetSimulationLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboResetSimulationSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(GazeboResetSimulationResponse, response);
        }
        return GazeboResetSimulationSuccess;
    }
    public boolean callGazeboApplyBodyWrench(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyBodyWrenchRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyBodyWrenchResponse response) {
        GazeboApplyBodyWrenchCondition = false;
        GazeboApplyBodyWrenchSuccess = false;
        GazeboApplyBodyWrench.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyBodyWrenchRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.ApplyBodyWrenchResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.ApplyBodyWrenchResponse mt) {
                GazeboApplyBodyWrenchResponse = mt;
                synchronized(GazeboApplyBodyWrenchLock) {
                    GazeboApplyBodyWrenchCondition = true;
                    GazeboApplyBodyWrenchSuccess = true;
                    GazeboApplyBodyWrenchLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboApplyBodyWrenchLock) {
                    GazeboApplyBodyWrenchCondition = true;
                    GazeboApplyBodyWrenchSuccess = false;
                    GazeboApplyBodyWrenchLock.notify();
                }
            }
        });
        
        synchronized(GazeboApplyBodyWrenchLock) {
            while(!GazeboApplyBodyWrenchCondition) {
                try {
                    GazeboApplyBodyWrenchLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboApplyBodyWrenchSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.ApplyBodyWrenchResponse.toAde(GazeboApplyBodyWrenchResponse, response);
        }
        return GazeboApplyBodyWrenchSuccess;
    }
    public boolean callGazeboGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response) {
        GazeboGetLoggersCondition = false;
        GazeboGetLoggersSuccess = false;
        GazeboGetLoggers.call(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest.toRos(request, node), 
                new ServiceResponseListener<roscpp.GetLoggersResponse>() {

            @Override
            public void onSuccess(roscpp.GetLoggersResponse mt) {
                GazeboGetLoggersResponse = mt;
                synchronized(GazeboGetLoggersLock) {
                    GazeboGetLoggersCondition = true;
                    GazeboGetLoggersSuccess = true;
                    GazeboGetLoggersLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetLoggersLock) {
                    GazeboGetLoggersCondition = true;
                    GazeboGetLoggersSuccess = false;
                    GazeboGetLoggersLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetLoggersLock) {
            while(!GazeboGetLoggersCondition) {
                try {
                    GazeboGetLoggersLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetLoggersSuccess) {
            edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse.toAde(GazeboGetLoggersResponse, response);
        }
        return GazeboGetLoggersSuccess;
    }
    public boolean callGazeboSpawnSdfModel(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelResponse response) {
        GazeboSpawnSdfModelCondition = false;
        GazeboSpawnSdfModelSuccess = false;
        GazeboSpawnSdfModel.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SpawnModelResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SpawnModelResponse mt) {
                GazeboSpawnSdfModelResponse = mt;
                synchronized(GazeboSpawnSdfModelLock) {
                    GazeboSpawnSdfModelCondition = true;
                    GazeboSpawnSdfModelSuccess = true;
                    GazeboSpawnSdfModelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSpawnSdfModelLock) {
                    GazeboSpawnSdfModelCondition = true;
                    GazeboSpawnSdfModelSuccess = false;
                    GazeboSpawnSdfModelLock.notify();
                }
            }
        });
        
        synchronized(GazeboSpawnSdfModelLock) {
            while(!GazeboSpawnSdfModelCondition) {
                try {
                    GazeboSpawnSdfModelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSpawnSdfModelSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelResponse.toAde(GazeboSpawnSdfModelResponse, response);
        }
        return GazeboSpawnSdfModelSuccess;
    }
    public boolean callGazeboSetLinkProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkPropertiesResponse response) {
        GazeboSetLinkPropertiesCondition = false;
        GazeboSetLinkPropertiesSuccess = false;
        GazeboSetLinkProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SetLinkPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SetLinkPropertiesResponse mt) {
                GazeboSetLinkPropertiesResponse = mt;
                synchronized(GazeboSetLinkPropertiesLock) {
                    GazeboSetLinkPropertiesCondition = true;
                    GazeboSetLinkPropertiesSuccess = true;
                    GazeboSetLinkPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetLinkPropertiesLock) {
                    GazeboSetLinkPropertiesCondition = true;
                    GazeboSetLinkPropertiesSuccess = false;
                    GazeboSetLinkPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetLinkPropertiesLock) {
            while(!GazeboSetLinkPropertiesCondition) {
                try {
                    GazeboSetLinkPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetLinkPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkPropertiesResponse.toAde(GazeboSetLinkPropertiesResponse, response);
        }
        return GazeboSetLinkPropertiesSuccess;
    }
    public boolean callGazeboSpawnGazeboModel(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelResponse response) {
        GazeboSpawnGazeboModelCondition = false;
        GazeboSpawnGazeboModelSuccess = false;
        GazeboSpawnGazeboModel.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SpawnModelResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SpawnModelResponse mt) {
                GazeboSpawnGazeboModelResponse = mt;
                synchronized(GazeboSpawnGazeboModelLock) {
                    GazeboSpawnGazeboModelCondition = true;
                    GazeboSpawnGazeboModelSuccess = true;
                    GazeboSpawnGazeboModelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSpawnGazeboModelLock) {
                    GazeboSpawnGazeboModelCondition = true;
                    GazeboSpawnGazeboModelSuccess = false;
                    GazeboSpawnGazeboModelLock.notify();
                }
            }
        });
        
        synchronized(GazeboSpawnGazeboModelLock) {
            while(!GazeboSpawnGazeboModelCondition) {
                try {
                    GazeboSpawnGazeboModelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSpawnGazeboModelSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelResponse.toAde(GazeboSpawnGazeboModelResponse, response);
        }
        return GazeboSpawnGazeboModelSuccess;
    }
    public boolean callGazeboPausePhysics(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
        GazeboPausePhysicsCondition = false;
        GazeboPausePhysicsSuccess = false;
        GazeboPausePhysics.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node), 
                new ServiceResponseListener<std_srvs.EmptyResponse>() {

            @Override
            public void onSuccess(std_srvs.EmptyResponse mt) {
                GazeboPausePhysicsResponse = mt;
                synchronized(GazeboPausePhysicsLock) {
                    GazeboPausePhysicsCondition = true;
                    GazeboPausePhysicsSuccess = true;
                    GazeboPausePhysicsLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboPausePhysicsLock) {
                    GazeboPausePhysicsCondition = true;
                    GazeboPausePhysicsSuccess = false;
                    GazeboPausePhysicsLock.notify();
                }
            }
        });
        
        synchronized(GazeboPausePhysicsLock) {
            while(!GazeboPausePhysicsCondition) {
                try {
                    GazeboPausePhysicsLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboPausePhysicsSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(GazeboPausePhysicsResponse, response);
        }
        return GazeboPausePhysicsSuccess;
    }
    public boolean callGazeboSetModelState(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelStateRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelStateResponse response) {
        GazeboSetModelStateCondition = false;
        GazeboSetModelStateSuccess = false;
        GazeboSetModelState.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelStateRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SetModelStateResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SetModelStateResponse mt) {
                GazeboSetModelStateResponse = mt;
                synchronized(GazeboSetModelStateLock) {
                    GazeboSetModelStateCondition = true;
                    GazeboSetModelStateSuccess = true;
                    GazeboSetModelStateLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetModelStateLock) {
                    GazeboSetModelStateCondition = true;
                    GazeboSetModelStateSuccess = false;
                    GazeboSetModelStateLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetModelStateLock) {
            while(!GazeboSetModelStateCondition) {
                try {
                    GazeboSetModelStateLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetModelStateSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelStateResponse.toAde(GazeboSetModelStateResponse, response);
        }
        return GazeboSetModelStateSuccess;
    }
    public boolean callGazeboSetJointProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetJointPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetJointPropertiesResponse response) {
        GazeboSetJointPropertiesCondition = false;
        GazeboSetJointPropertiesSuccess = false;
        GazeboSetJointProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetJointPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SetJointPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SetJointPropertiesResponse mt) {
                GazeboSetJointPropertiesResponse = mt;
                synchronized(GazeboSetJointPropertiesLock) {
                    GazeboSetJointPropertiesCondition = true;
                    GazeboSetJointPropertiesSuccess = true;
                    GazeboSetJointPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetJointPropertiesLock) {
                    GazeboSetJointPropertiesCondition = true;
                    GazeboSetJointPropertiesSuccess = false;
                    GazeboSetJointPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetJointPropertiesLock) {
            while(!GazeboSetJointPropertiesCondition) {
                try {
                    GazeboSetJointPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetJointPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetJointPropertiesResponse.toAde(GazeboSetJointPropertiesResponse, response);
        }
        return GazeboSetJointPropertiesSuccess;
    }
    public boolean callGazeboClearBodyWrenches(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.BodyRequestRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.BodyRequestResponse response) {
        GazeboClearBodyWrenchesCondition = false;
        GazeboClearBodyWrenchesSuccess = false;
        GazeboClearBodyWrenches.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.BodyRequestRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.BodyRequestResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.BodyRequestResponse mt) {
                GazeboClearBodyWrenchesResponse = mt;
                synchronized(GazeboClearBodyWrenchesLock) {
                    GazeboClearBodyWrenchesCondition = true;
                    GazeboClearBodyWrenchesSuccess = true;
                    GazeboClearBodyWrenchesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboClearBodyWrenchesLock) {
                    GazeboClearBodyWrenchesCondition = true;
                    GazeboClearBodyWrenchesSuccess = false;
                    GazeboClearBodyWrenchesLock.notify();
                }
            }
        });
        
        synchronized(GazeboClearBodyWrenchesLock) {
            while(!GazeboClearBodyWrenchesCondition) {
                try {
                    GazeboClearBodyWrenchesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboClearBodyWrenchesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.BodyRequestResponse.toAde(GazeboClearBodyWrenchesResponse, response);
        }
        return GazeboClearBodyWrenchesSuccess;
    }
    public boolean callGazeboResetWorld(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
        GazeboResetWorldCondition = false;
        GazeboResetWorldSuccess = false;
        GazeboResetWorld.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node), 
                new ServiceResponseListener<std_srvs.EmptyResponse>() {

            @Override
            public void onSuccess(std_srvs.EmptyResponse mt) {
                GazeboResetWorldResponse = mt;
                synchronized(GazeboResetWorldLock) {
                    GazeboResetWorldCondition = true;
                    GazeboResetWorldSuccess = true;
                    GazeboResetWorldLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboResetWorldLock) {
                    GazeboResetWorldCondition = true;
                    GazeboResetWorldSuccess = false;
                    GazeboResetWorldLock.notify();
                }
            }
        });
        
        synchronized(GazeboResetWorldLock) {
            while(!GazeboResetWorldCondition) {
                try {
                    GazeboResetWorldLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboResetWorldSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(GazeboResetWorldResponse, response);
        }
        return GazeboResetWorldSuccess;
    }
    public boolean callGazeboSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response) {
        GazeboSetParametersCondition = false;
        GazeboSetParametersSuccess = false;
        GazeboSetParameters.call(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest.toRos(request, node), 
                new ServiceResponseListener<dynamic_reconfigure.ReconfigureResponse>() {

            @Override
            public void onSuccess(dynamic_reconfigure.ReconfigureResponse mt) {
                GazeboSetParametersResponse = mt;
                synchronized(GazeboSetParametersLock) {
                    GazeboSetParametersCondition = true;
                    GazeboSetParametersSuccess = true;
                    GazeboSetParametersLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetParametersLock) {
                    GazeboSetParametersCondition = true;
                    GazeboSetParametersSuccess = false;
                    GazeboSetParametersLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetParametersLock) {
            while(!GazeboSetParametersCondition) {
                try {
                    GazeboSetParametersLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetParametersSuccess) {
            edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse.toAde(GazeboSetParametersResponse, response);
        }
        return GazeboSetParametersSuccess;
    }
    public boolean callGazeboSetModelConfiguration(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelConfigurationRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelConfigurationResponse response) {
        GazeboSetModelConfigurationCondition = false;
        GazeboSetModelConfigurationSuccess = false;
        GazeboSetModelConfiguration.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelConfigurationRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SetModelConfigurationResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SetModelConfigurationResponse mt) {
                GazeboSetModelConfigurationResponse = mt;
                synchronized(GazeboSetModelConfigurationLock) {
                    GazeboSetModelConfigurationCondition = true;
                    GazeboSetModelConfigurationSuccess = true;
                    GazeboSetModelConfigurationLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetModelConfigurationLock) {
                    GazeboSetModelConfigurationCondition = true;
                    GazeboSetModelConfigurationSuccess = false;
                    GazeboSetModelConfigurationLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetModelConfigurationLock) {
            while(!GazeboSetModelConfigurationCondition) {
                try {
                    GazeboSetModelConfigurationLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetModelConfigurationSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetModelConfigurationResponse.toAde(GazeboSetModelConfigurationResponse, response);
        }
        return GazeboSetModelConfigurationSuccess;
    }
    public boolean callGazeboSetPhysicsProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetPhysicsPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetPhysicsPropertiesResponse response) {
        GazeboSetPhysicsPropertiesCondition = false;
        GazeboSetPhysicsPropertiesSuccess = false;
        GazeboSetPhysicsProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetPhysicsPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SetPhysicsPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SetPhysicsPropertiesResponse mt) {
                GazeboSetPhysicsPropertiesResponse = mt;
                synchronized(GazeboSetPhysicsPropertiesLock) {
                    GazeboSetPhysicsPropertiesCondition = true;
                    GazeboSetPhysicsPropertiesSuccess = true;
                    GazeboSetPhysicsPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetPhysicsPropertiesLock) {
                    GazeboSetPhysicsPropertiesCondition = true;
                    GazeboSetPhysicsPropertiesSuccess = false;
                    GazeboSetPhysicsPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetPhysicsPropertiesLock) {
            while(!GazeboSetPhysicsPropertiesCondition) {
                try {
                    GazeboSetPhysicsPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetPhysicsPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetPhysicsPropertiesResponse.toAde(GazeboSetPhysicsPropertiesResponse, response);
        }
        return GazeboSetPhysicsPropertiesSuccess;
    }
    public boolean callGazeboGetWorldProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetWorldPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetWorldPropertiesResponse response) {
        GazeboGetWorldPropertiesCondition = false;
        GazeboGetWorldPropertiesSuccess = false;
        GazeboGetWorldProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetWorldPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.GetWorldPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.GetWorldPropertiesResponse mt) {
                GazeboGetWorldPropertiesResponse = mt;
                synchronized(GazeboGetWorldPropertiesLock) {
                    GazeboGetWorldPropertiesCondition = true;
                    GazeboGetWorldPropertiesSuccess = true;
                    GazeboGetWorldPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetWorldPropertiesLock) {
                    GazeboGetWorldPropertiesCondition = true;
                    GazeboGetWorldPropertiesSuccess = false;
                    GazeboGetWorldPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetWorldPropertiesLock) {
            while(!GazeboGetWorldPropertiesCondition) {
                try {
                    GazeboGetWorldPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetWorldPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetWorldPropertiesResponse.toAde(GazeboGetWorldPropertiesResponse, response);
        }
        return GazeboGetWorldPropertiesSuccess;
    }
    public boolean callGazeboSetLinkState(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkStateRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkStateResponse response) {
        GazeboSetLinkStateCondition = false;
        GazeboSetLinkStateSuccess = false;
        GazeboSetLinkState.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkStateRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SetLinkStateResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SetLinkStateResponse mt) {
                GazeboSetLinkStateResponse = mt;
                synchronized(GazeboSetLinkStateLock) {
                    GazeboSetLinkStateCondition = true;
                    GazeboSetLinkStateSuccess = true;
                    GazeboSetLinkStateLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSetLinkStateLock) {
                    GazeboSetLinkStateCondition = true;
                    GazeboSetLinkStateSuccess = false;
                    GazeboSetLinkStateLock.notify();
                }
            }
        });
        
        synchronized(GazeboSetLinkStateLock) {
            while(!GazeboSetLinkStateCondition) {
                try {
                    GazeboSetLinkStateLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSetLinkStateSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SetLinkStateResponse.toAde(GazeboSetLinkStateResponse, response);
        }
        return GazeboSetLinkStateSuccess;
    }
    public boolean callGazeboUnpausePhysics(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response) {
        GazeboUnpausePhysicsCondition = false;
        GazeboUnpausePhysicsSuccess = false;
        GazeboUnpausePhysics.call(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest.toRos(request, node), 
                new ServiceResponseListener<std_srvs.EmptyResponse>() {

            @Override
            public void onSuccess(std_srvs.EmptyResponse mt) {
                GazeboUnpausePhysicsResponse = mt;
                synchronized(GazeboUnpausePhysicsLock) {
                    GazeboUnpausePhysicsCondition = true;
                    GazeboUnpausePhysicsSuccess = true;
                    GazeboUnpausePhysicsLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboUnpausePhysicsLock) {
                    GazeboUnpausePhysicsCondition = true;
                    GazeboUnpausePhysicsSuccess = false;
                    GazeboUnpausePhysicsLock.notify();
                }
            }
        });
        
        synchronized(GazeboUnpausePhysicsLock) {
            while(!GazeboUnpausePhysicsCondition) {
                try {
                    GazeboUnpausePhysicsLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboUnpausePhysicsSuccess) {
            edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse.toAde(GazeboUnpausePhysicsResponse, response);
        }
        return GazeboUnpausePhysicsSuccess;
    }
    public boolean callGazeboSpawnUrdfModel(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelResponse response) {
        GazeboSpawnUrdfModelCondition = false;
        GazeboSpawnUrdfModelSuccess = false;
        GazeboSpawnUrdfModel.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.SpawnModelResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.SpawnModelResponse mt) {
                GazeboSpawnUrdfModelResponse = mt;
                synchronized(GazeboSpawnUrdfModelLock) {
                    GazeboSpawnUrdfModelCondition = true;
                    GazeboSpawnUrdfModelSuccess = true;
                    GazeboSpawnUrdfModelLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboSpawnUrdfModelLock) {
                    GazeboSpawnUrdfModelCondition = true;
                    GazeboSpawnUrdfModelSuccess = false;
                    GazeboSpawnUrdfModelLock.notify();
                }
            }
        });
        
        synchronized(GazeboSpawnUrdfModelLock) {
            while(!GazeboSpawnUrdfModelCondition) {
                try {
                    GazeboSpawnUrdfModelLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboSpawnUrdfModelSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.SpawnModelResponse.toAde(GazeboSpawnUrdfModelResponse, response);
        }
        return GazeboSpawnUrdfModelSuccess;
    }
    public boolean callGazeboClearJointForces(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.JointRequestRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.JointRequestResponse response) {
        GazeboClearJointForcesCondition = false;
        GazeboClearJointForcesSuccess = false;
        GazeboClearJointForces.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.JointRequestRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.JointRequestResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.JointRequestResponse mt) {
                GazeboClearJointForcesResponse = mt;
                synchronized(GazeboClearJointForcesLock) {
                    GazeboClearJointForcesCondition = true;
                    GazeboClearJointForcesSuccess = true;
                    GazeboClearJointForcesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboClearJointForcesLock) {
                    GazeboClearJointForcesCondition = true;
                    GazeboClearJointForcesSuccess = false;
                    GazeboClearJointForcesLock.notify();
                }
            }
        });
        
        synchronized(GazeboClearJointForcesLock) {
            while(!GazeboClearJointForcesCondition) {
                try {
                    GazeboClearJointForcesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboClearJointForcesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.JointRequestResponse.toAde(GazeboClearJointForcesResponse, response);
        }
        return GazeboClearJointForcesSuccess;
    }
    public boolean callGazeboGetJointProperties(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetJointPropertiesRequest request, edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetJointPropertiesResponse response) {
        GazeboGetJointPropertiesCondition = false;
        GazeboGetJointPropertiesSuccess = false;
        GazeboGetJointProperties.call(edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetJointPropertiesRequest.toRos(request, node), 
                new ServiceResponseListener<gazebo_msgs.GetJointPropertiesResponse>() {

            @Override
            public void onSuccess(gazebo_msgs.GetJointPropertiesResponse mt) {
                GazeboGetJointPropertiesResponse = mt;
                synchronized(GazeboGetJointPropertiesLock) {
                    GazeboGetJointPropertiesCondition = true;
                    GazeboGetJointPropertiesSuccess = true;
                    GazeboGetJointPropertiesLock.notify();
                }
            }

            @Override
            public void onFailure(org.ros.exception.RemoteException re) {
                synchronized(GazeboGetJointPropertiesLock) {
                    GazeboGetJointPropertiesCondition = true;
                    GazeboGetJointPropertiesSuccess = false;
                    GazeboGetJointPropertiesLock.notify();
                }
            }
        });
        
        synchronized(GazeboGetJointPropertiesLock) {
            while(!GazeboGetJointPropertiesCondition) {
                try {
                    GazeboGetJointPropertiesLock.wait();
                } catch (InterruptedException e) {
                }
            }
        }
        if (GazeboGetJointPropertiesSuccess) {
            edu.tufts.hrilab.diarcros.msg.gazebo_msgs.GetJointPropertiesResponse.toAde(GazeboGetJointPropertiesResponse, response);
        }
        return GazeboGetJointPropertiesSuccess;
    }
}
    
