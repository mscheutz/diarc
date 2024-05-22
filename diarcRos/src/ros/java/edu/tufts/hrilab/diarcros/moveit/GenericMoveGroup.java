/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.moveit;

import org.ros.exception.RosException;

public interface GenericMoveGroup {
  /* TODO:
      There's a handful of functions that were added by hand to the auto-generated functions,
      and 9 times out of 10 they're identical to each other. This interface should probably be
      made into an abstract class to avoid unnecessary code duplication.
   */

  // These functions need to be here for the MoveItComponent to function.
  String getMoveGroupRosVersion();
  boolean callComputeCartesianPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetCartesianPathRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetCartesianPathResponse response);
  boolean callComputeFk(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionFKRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionFKResponse response);
  boolean callComputeIk(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionIKRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPositionIKResponse response);
  boolean callExecuteKinematicPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteKnownTrajectoryRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.ExecuteKnownTrajectoryResponse response);
  boolean callGetPlanningScene(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlanningSceneRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetPlanningSceneResponse response);
  boolean callPlanKinematicPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetMotionPlanRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetMotionPlanResponse response);
  boolean executeKinematicPath(edu.tufts.hrilab.diarcros.msg.moveit_msgs.RobotTrajectory rt) throws InterruptedException, RosException;
  boolean callClearOctomap(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response);
  boolean isConnected();
  void sendAttachedCollisionObject(edu.tufts.hrilab.diarcros.msg.moveit_msgs.AttachedCollisionObject msg);
  void sendCollisionObject(edu.tufts.hrilab.diarcros.msg.moveit_msgs.CollisionObject msg);
  void sendPlanningScene(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningScene msg);
  void sendPointCloud(edu.tufts.hrilab.diarcros.msg.sensor_msgs.PointCloud2 msg);
  void shutdown();
  void waitForNode();

  // These functions exist in (most) of the MoveGroups, but are not needed in the MoveItComponent.
  // They are commented out to avoid the need to implement them everywhere by hand.

//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupOmplParameterUpdates();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupPlanExecutionParameterUpdates();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupPlanningSceneMonitorParameterUpdates();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupSenseForPlanParameterUpdates();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.Config getMoveGroupTrajectoryExecutionParameterUpdates();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupOmplParameterDescriptions();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupPlanExecutionParameterDescriptions();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupPlanningSceneMonitorParameterDescriptions();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupSenseForPlanParameterDescriptions();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ConfigDescription getMoveGroupTrajectoryExecutionParameterDescriptions();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.moveit_msgs.DisplayTrajectory getMoveGroupDisplayPlannedPath();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningScene getMoveGroupMonitoredPlanningScene();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Log getRosout();
//  /*synchronized*/ edu.tufts.hrilab.diarcros.msg.visualization_msgs.MarkerArray getMoveGroupDisplayContacts();
//  SimpleClientGoalState getLeftControlMsgsFollowJointTrajectoryState();
//  SimpleClientGoalState getMoveitMsgsExecuteTrajectoryState();
//  SimpleClientGoalState getMoveitMsgsMoveGroupState();
//  SimpleClientGoalState getMoveitMsgsPickupState();
//  SimpleClientGoalState getMoveitMsgsPlaceState();
//  SimpleClientGoalState getRightControlMsgsFollowJointTrajectoryState();
//  boolean callCheckStateValidity(edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetStateValidityRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetStateValidityResponse response);
//  boolean callClearOctomap(edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest request, edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse response);
//  boolean callMoveGroupGetLoggers(edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.GetLoggersResponse response);
//  boolean callMoveGroupLoadMap(edu.tufts.hrilab.diarcros.msg.moveit_msgs.LoadMapRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.LoadMapResponse response);
//  boolean callMoveGroupOmplSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response);
//  boolean callMoveGroupPlanExecutionSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response);
//  boolean callMoveGroupPlanningSceneMonitorSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response);
//  boolean callMoveGroupSaveMap(edu.tufts.hrilab.diarcros.msg.moveit_msgs.SaveMapRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.SaveMapResponse response);
//  boolean callMoveGroupSenseForPlanSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response);
//  boolean callMoveGroupSetLoggerLevel(edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelRequest request, edu.tufts.hrilab.diarcros.msg.roscpp.SetLoggerLevelResponse response);
//  boolean callMoveGroupTrajectoryExecutionSetParameters(edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureRequest request, edu.tufts.hrilab.diarcros.msg.dynamic_reconfigure.ReconfigureResponse response);
//  boolean callQueryPlannerInterface(edu.tufts.hrilab.diarcros.msg.moveit_msgs.QueryPlannerInterfacesRequest request, edu.tufts.hrilab.diarcros.msg.moveit_msgs.QueryPlannerInterfacesResponse response);
//  boolean isReady();
//  edu.tufts.hrilab.diarcros.msg.Time getCurrentTime();
//  void cancelAllLeftControlMsgsFollowJointTrajectoryGoals();
//  void cancelAllMoveitMsgsExecuteTrajectoryGoals();
//  void cancelAllMoveitMsgsMoveGroupGoals();
//  void cancelAllMoveitMsgsPickupGoals();
//  void cancelAllMoveitMsgsPlaceGoals();
//  void cancelAllRightArmControlMsgsFollowJointTrajectoryGoals();
//  void cancelLeftControlMsgsFollowJointTrajectoryGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time);
//  void cancelMoveitMsgsExecuteTrajectoryGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time);
//  void cancelMoveitMsgsMoveGroupGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time);
//  void cancelMoveitMsgsPickupGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time);
//  void cancelMoveitMsgsPlaceGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time);
//  void cancelRightControlMsgsFollowJointTrajectoryGoalsAtAndBeforeTime(edu.tufts.hrilab.diarcros.msg.Time time);
//  void sendClock(edu.tufts.hrilab.diarcros.msg.rosgraph_msgs.Clock msg);
//  void sendHardwareJointStates(edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState msg);
//  void sendPlanningSceneWorld(edu.tufts.hrilab.diarcros.msg.moveit_msgs.PlanningSceneWorld msg);
//  void sendTf(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg);
//  void sendTfStatic(edu.tufts.hrilab.diarcros.msg.tf2_msgs.TFMessage msg);
//  void sendTrajectoryExecutionEvent(edu.tufts.hrilab.diarcros.msg.std_msgs.String msg);
}
