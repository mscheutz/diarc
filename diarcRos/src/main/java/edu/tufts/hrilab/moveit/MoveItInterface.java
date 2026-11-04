/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.moveit;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.interfaces.ArmInterface;
import edu.tufts.hrilab.interfaces.LearningInterface;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public interface MoveItInterface extends ArmInterface, LearningInterface {

    @TRADEService
    @Action
    @Deprecated
    boolean moveToJointPositions(String[] joint_names, double[] positions, double[] velocities, double[] efforts);

    @TRADEService
    @Action
    @Deprecated
    boolean moveToJointPositions(String[] jointNames, double[] positions);

    /**
     * Set the position of a single joint. All remaining joints will remain in the same position.
     *
     * @param joint    The name of the joint to manipulate, as found in `names` of /joint_states
     * @param position The goal position, as viewed in /joint_states
     * @return true on success, false on failure. This function blocks.
     */
    @TRADEService
    @Action
    @Deprecated
    boolean moveToJointPosition(String joint, double position);

    /**
     * Turn on carrying orientation constraints. This will apply to all moveTo and goToPose methods.
     */
    @TRADEService
    @Action
    void enableCarryingConstraints();

    /**
     * Turn off carrying orientation constraints.
     */
    @TRADEService
    @Action
    void disableCarryingConstraints();

    @TRADEService
    @Action
    Justification pointTo(String groupName, Point3d location);

    @TRADEService
    @Action
    Justification moveToCartesian(String groupName, Point3d point, Quat4d orientation);

    /**
     * Publish the compressed depth data to ROS after being decompressed and converted to proper point cloud form.
     *
     * @param depthData A byte array containing the point cloud data.
     * @return true on success.
     */
    @TRADEService
    boolean publishPointCloudToRos(byte[] depthData);

}
