/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.movebase;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarcros.cmd_vel.CmdVelPublisher;
import edu.tufts.hrilab.diarcros.laserscan.LaserScanSubscriber;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;
import edu.tufts.hrilab.diarcros.util.Convert;
import edu.tufts.hrilab.util.Util;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import javax.vecmath.Quat4d;

import static edu.tufts.hrilab.diarcros.util.Convert.YAW;

public class MoveTowardsEnabler {
    private static Logger log = LoggerFactory.getLogger(MoveTowardsEnabler.class);
    private final LaserScanSubscriber laserScanSubscriber;
    private final CmdVelPublisher cmdVelPublisher;
    private static final double OFFSET_STEP_DISTANCE = (2.0 / 5.0);  // When computing the offset distance, how far away should we be (in meters)?
    private static final double APPROACH_END_DISTANCE = 0.25;
    private static final double TAU = 2 * Math.PI; // τ (AKA 2π)

    public MoveTowardsEnabler() {
        laserScanSubscriber = new LaserScanSubscriber("/base_scan");
        // rangeSubscriber = new RangeSubscriber("/spot/camera/hand_depth/range");
        cmdVelPublisher = new CmdVelPublisher();
        log.debug("MoveTowardsEnabler waiting on laser scan data...");
        laserScanSubscriber.waitForNode();
        // rangeSubscriber.waitForNode();
        log.debug("Got the laser scan node. Now waiting on cmd_vel...");
        cmdVelPublisher.waitForNode();
        log.info("MoveTowardsEnabler ready to go!");
    }

    /**
     * Calculate the average lidar distance within specified angle of forward-looking laser beam.
     * @param angle angle to include in calculation (radians)
     * @return
     */
    @TRADEService
    public double getLidarDistanceWithinAngle(double angle) {
        if (!laserScanSubscriber.isNodeReady()) {
            log.error("Not receiving laser scan data!");
            throw new NullPointerException();
        }

        try {
            float[] ls = laserScanSubscriber.getScan().getRanges();
            int index = (int) Math.floor(ls.length/2.0);
            int indexOffset = (int)Math.round(angle/laserScanSubscriber.getScan().getAngleIncrement()) / 2;
            double sum = 0;
            int num = 0;
            for (int i = index - indexOffset; i <= index + indexOffset; i++) {
                double curr = ls[i];
                if (!(Double.isInfinite(curr) || Double.isNaN(curr))) {
                    sum += curr;
                    num++;
                }
            }

            double result = 0;
            if (num > 0) {
                result = sum / num;
            }
            return result;
        } catch (NullPointerException e){
             log.error("Requesting laser scan data is null!", e);
        }
        return Double.NaN;
    }

    /**
     * Slowly work our way forward (via cmd_vel) while constantly
     * checking the LIDAR for any potential obstacles.
     * @return
     */
    @TRADEService
    public Justification moveTowards() {
        try {
            log.info("Got 'moveTowards' request.");
            boolean keepMovingTowards = false;
            double distance_limit = 1; // don't go beyond 1 meter
            double estimated_travel = 0;
            double hz = 5.0;
            while (estimated_travel < distance_limit) {
                // guesstimated and hard-coded x-val for now
                double lidar_distance = getLidarDistanceWithinAngle(0.2);
                keepMovingTowards = lidar_distance > APPROACH_END_DISTANCE;
                if (!keepMovingTowards) {
                    log.info("Done moving forward. Lidar distance: " + lidar_distance);
                    break;
                }

                double movement_speed = 0.25;
                log.debug("Moving forwards at "+movement_speed+"m/s. Laser distance: "+String.format("%.3f",lidar_distance)+", estimated travel: "+String.format("%.3f", estimated_travel));
                cmdVelPublisher.sendLinearCmdVel(movement_speed, 0, 0);
                Util.Sleep((long) (1 / hz * 1000)); // publish at about 30hz
                estimated_travel += movement_speed * (1/hz);
            }
            log.info("Done with 'moveTowards' request "+(keepMovingTowards? " (exceeded upper travel limit)" : "(within LIDAR distance)"));
        } finally {
            // make sure we're not moving when we're all done here
            cmdVelPublisher.sendStopCmdVel();
        }

        return new ConditionJustification(true);
    }

    private double dist(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }

    // @Action
    @TRADEService
    public boolean forwardMove(double move_speed, double distance_lim, MoveBaseComponent movebase) {
        // 'approaching' is slowly working our way forward (via cmd_vel) while constantly
        // checking the LIDAR for any potential obstacles.
        boolean ret = false;

        try {
            Point startPos = movebase.getPose().getPosition();
            log.info("Got 'forwardMove' request: " + distance_lim + " at " + move_speed + "m/s");
            boolean keepMovingTowards = false;
            double distance_limit = distance_lim; // don't go beyond 1 meter
            double estimated_travel = 0;
            double timeEstimate = distance_lim / move_speed;
            double timeout = 2 * timeEstimate + 5;
            log.info("Timeout: " + timeout);
            double hz = 30.0;
            int offset = 35;
            long start = System.currentTimeMillis();
            Point currPos = movebase.getPose().getPosition();
            while (dist(startPos, currPos) < Math.abs(distance_lim)) {
                long currTime = System.currentTimeMillis();
                log.info("Elapsed: " + ((currTime - start) / 1000.0));
                if ((currTime - start) / 1000.0 > timeout) {
                    log.info("Spot move timeout");
                    return false;
                }
                currPos = movebase.getPose().getPosition();
                // guesstimated and hard-coded x-val for now
                double lidar_distance = 0;//getRangeDistance();

                if (move_speed < 0) {
                    lidar_distance = getLidarDistanceWithinAngle(0.2);
                    Util.Sleep((long)((1 / hz) * 1000));
                }
                log.info("lid" + lidar_distance);
                keepMovingTowards = lidar_distance > 0.8;
                if (!keepMovingTowards) {
                    ret = true;
                    break;
                }
                double movement_speed = move_speed;
                log.debug("Moving forwards at "+movement_speed+"m/s. Laser distance: "+String.format("%.3f",lidar_distance)+", estimated travel: "+String.format("%.3f", estimated_travel));
                cmdVelPublisher.sendLinearCmdVel(movement_speed, 0, 0);
            }
            log.info("Done with 'spotMove' request "+(keepMovingTowards? " (exceeded upper travel limit)" : "(within LIDAR distance)"));
        } finally {
            // make sure we're not moving when we're all done here
            cmdVelPublisher.sendStopCmdVel();
        }

        return ret;
    }

    /**
     * Rotates the robot the specified degrees of rotation or shorter equivalent rotation
     *
     * @param rot_amount the amount to rotate in radians (counterclockwise (CCW))
     * @return Condition Justification if the function was a success or s failure
     */
    @TRADEService
    @Action
    public Justification rotate(double rot_amount) {
        log.debug("Starting [rotate]");

        // processing input
        rot_amount = (rot_amount % TAU + TAU) % TAU; // Restrict to between 0 and 2π
        rot_amount = (rot_amount > Math.PI) ? (rot_amount - TAU) : rot_amount; // Convert to between -π and π
        double abs_rot_amount = Math.abs(rot_amount);

        // set up
        double hz = 10.0;
        double cycleTime = 1 / hz;
        double rot_speed = 0.5;
        boolean isCCW = rot_amount > 0;
        rot_speed = isCCW ? rot_speed : -rot_speed; // + if CCW, - if CW

        double startOrient = getOrientation();
        double currOrient = startOrient;
        double pastOrient = currOrient;

        try {
            double abs_estimated_rot = 0;
            while (abs_estimated_rot < abs_rot_amount) {
                cmdVelPublisher.sendAngularCmdVel(0, 0, rot_speed);

                pastOrient = currOrient;
                currOrient = getOrientation();
                if (isCCW && pastOrient > 0 && currOrient < 0) { // if the orientation switched from π to -π moving CCW
                    currOrient += TAU;
                } else if (!isCCW && pastOrient < 0 && currOrient > 0) { // if the orientation switched from -π to π moving CW
                    currOrient -= TAU;
                }
                abs_estimated_rot = Math.abs(currOrient - startOrient);

                log.debug("Rotating CCW?: " + isCCW +
                        " Current Orientation: " + String.format("%.3f", currOrient) +
                        " Estimated rotation: " + String.format("%.3f", abs_estimated_rot));

                Util.Sleep((long) (cycleTime * 1000));
            }
        } finally {
            cmdVelPublisher.sendStopCmdVel(); // Ensure no movement when finished
        }
        log.debug("Finished [rotate]");
        return new ConditionJustification(true);
    }

    /**
     * Gets the CCW rotational orientation of the robot in radians
     *
     * @return The CCW rotational orientation of the robot in radians
     */
    private double getOrientation() {
        double orient = 0;
        try {
            double[] pose = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPoseGlobalQuat")).call(double[].class);
            Quat4d poseQuat = new Quat4d(Arrays.copyOfRange(pose, 2, 6));
            orient = edu.tufts.hrilab.util.Convert.convertToEuler(poseQuat).getY();
        } catch (TRADEException e) {
            log.error("Error calling getPose.", e);
        }
        return orient;
    }

    @TRADEService
    public double[] computeOffset(double[] pose) {
        if (pose.length != 7) {
            log.error("The length of the set of numbers you gave to computeOffset was != 7: input should correspond to x, y, z, quat_x, quat_y, quat_z, quat_w (in ros format).");
            return null;
        }
        Pose p = new Pose(new Point(pose[0], pose[1], pose[2]), new Quaternion(pose[3], pose[4], pose[5], pose[6]));
        double[] e = Convert.quaternionToEuler(p.getOrientation());
        double yaw = e[YAW];
        double x_component = Math.cos(yaw);
        double y_component = Math.sin(yaw);

        p.getPosition().setX(p.getPosition().getX() + (x_component * OFFSET_STEP_DISTANCE));
        p.getPosition().setY(p.getPosition().getY() + (y_component * OFFSET_STEP_DISTANCE));
        return new double[] {
                p.getPosition().getX(),
                p.getPosition().getY(),
                p.getPosition().getZ(),
                p.getOrientation().getX(),
                p.getOrientation().getY(),
                p.getOrientation().getZ(),
                p.getOrientation().getW()
        };
    }
}
