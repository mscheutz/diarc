/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import edu.tufts.hrilab.ava.message.response.Obstacle;
import edu.tufts.hrilab.util.Convert;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.List;

/**
 * Helper class to "track" target obstacle and calculate velocities for robot following behavior.
 */
public class Follower {
  /**
   * Target obstacle from previous iteration. Used to "track" the target obstacle in the case of several obstacles.
   */
  private Obstacle lastTarget = null;
  private double lastTargetTheta = Double.MAX_VALUE;
  private double lastTargetDistance = Double.MAX_VALUE;

  /**
   * Clear the last target.
   */
  public void reset() {
    lastTarget = null;
    lastTargetTheta = Double.MAX_VALUE;
    lastTargetDistance = Double.MAX_VALUE;
  }

  /**
   * Select the obstacle to follow.
   *
   * @return
   */
  public Obstacle getObstacleToFollow(Pair<Point3d, Quat4d> currPose, List<Obstacle> obstacles) {
    if (obstacles.isEmpty()) {
      return null;
    }

    Obstacle target = null;
    double targetTheta = Double.MAX_VALUE;
    double targetDist = Double.MAX_VALUE;
    double targetScore = Double.MAX_VALUE;
    for (Obstacle currObstacle : obstacles) {

      // calculate vector and angle from robot to obstacle
      Vector3d obsDirVec = getObstacleVector(currPose, currObstacle);
      double currObsTheta = getTargetAngle(currPose, obsDirVec);

      // choose target based on theta and distance from robot criteria
      if (lastTarget == null) {
        // if not previous target, prefer obstacle directly in front of ava
        if (Math.abs(currObsTheta) < Math.abs(targetTheta)) {
          target = currObstacle;
          targetTheta = currObsTheta;
          targetDist = obsDirVec.length();
        }
      } else {
        // prefer closest to previous target
        double currObsDist = obsDirVec.length();
        double currObsScore = Math.abs(lastTargetTheta - currObsTheta) + Math.abs(lastTargetDistance - currObsDist);
        if (currObsScore < targetScore) {
          target = currObstacle;
          targetTheta = currObsTheta;
          targetDist = currObsDist;
          targetScore = currObsScore;
        }
      }
    }

    // update last target info
    lastTarget = target;
    lastTargetTheta = targetTheta;
    lastTargetDistance = targetDist;
    return target;
  }

  /**
   * Use the robot pose and target obstacle location to calculate translation and rotation velocities.
   *
   * @param currPose - current robot pose
   * @param target   - target obstacle to move towards
   * @return {transVelocity (m/s), rotVelocity(rad/s)}
   */

  public float[] calculateFollowVelocity(Pair<Point3d, Quat4d> currPose, Obstacle target) {

    // calculate vector and angle from robot to obstacle
    Vector3d obsDirVec = getObstacleVector(currPose, target);
    double targetTheta = getTargetAngle(currPose, obsDirVec);

    // calculate velocities
    double transVel = getTransVel(obsDirVec);
    double rotVel = getRotVel(targetTheta);

    return new float[]{(float) transVel, (float) rotVel};
  }

  /**
   * Calculate vector from current pose to target.
   *
   * @param currPose current robot pose
   * @param target   target Obstacle
   * @return
   */
  private Vector3d getObstacleVector(Pair<Point3d, Quat4d> currPose, Obstacle target) {
    Vector3d obsDirVec = new Vector3d();
    obsDirVec.sub(new Vector3d(Double.valueOf(target.position.x), Double.valueOf(target.position.y), 0.0), currPose.getLeft());
    return obsDirVec;
  }

  /**
   * Calculate angle (radians) between robot heading and vector to obstacle.
   *
   * @param currPose  current robot pose
   * @param obsDirVec vector from robot position to obstacle position
   * @return
   */
  private double getTargetAngle(Pair<Point3d, Quat4d> currPose, Vector3d obsDirVec) {
    Matrix4d poseTransform = Convert.convertToMatrix4d(currPose);
    Vector3d robotDirVec = new Vector3d(1, 0, 0);
    poseTransform.transform(robotDirVec);

    double targetTheta = robotDirVec.angle(obsDirVec); // angle between robot and obstacle (0,PI)

    // find sign of angle
    Vector3d cross = new Vector3d();
    cross.cross(robotDirVec, obsDirVec);
    Vector3d normal = new Vector3d(0, 0, 1);
    if (normal.dot(cross) < 0) { // Or > 0
      targetTheta = -targetTheta;
    }

    return targetTheta;
  }

  /**
   * Helper method to calculate the translational velocity for the follow actions.
   *
   * @param errorVector - vector from robot pose to target obstacle
   * @return
   */
  private double getTransVel(Vector3d errorVector) {
    // set constraints for velocity calculation
    float targetTransVel = 1.0f; // m/s
    float maxTransVel = 1.5f; // m/s
    float minTransVel = 0.4f; // m/s
    float targetFollowDist = 2.0f; // m
    float minFollowDist = 1.0f; // m -- don't get closer than this

    // calculate linear translational velocity based on linear params
    float m = (targetTransVel - minTransVel) / (targetFollowDist - minFollowDist); // line slope
    float b = minTransVel - m; // x intercept
    double transVel = m * errorVector.length() + b;

    // don't get closer than minFollowDist
    if (errorVector.length() < minFollowDist) {
      transVel = 0;
    }

    // apply min/max constraints
    if (transVel > maxTransVel) {
      transVel = maxTransVel;
    } else if (transVel < minTransVel) {
      transVel = 0;
    }

    return transVel;
  }

  /**
   * Helper method to calculate rotational velocity for follow action based on the heading angle between the
   * robot and target obstacle (e.g., 0, directly in front, + to robot's left, - to robot's right).
   *
   * @param targetAngle angle in radians (-PI,PI)
   * @return
   */
  private double getRotVel(double targetAngle) {
    float maxRotVel = 2.0f; // rad/s
    float minRotVel = 0.5f; // rad/s
    double startMaxVelAngle = Math.PI / 2.0; // rad
    double zeroAngleEps = 10.0 * (Math.PI / 180.0); // rad
    double m_rot = (maxRotVel - minRotVel) / (startMaxVelAngle - zeroAngleEps);
    double b_rot = maxRotVel - m_rot * startMaxVelAngle;
    double rotVel = m_rot * Math.abs(targetAngle) + b_rot; // abs so rotVel is always positive (will correct as last step)

    // apply min/max constraints
    if (rotVel > maxRotVel) {
      rotVel = maxRotVel;
    } else if (rotVel < minRotVel) {
      rotVel = minRotVel;
    }

    if (Math.abs(targetAngle) < zeroAngleEps) {
      rotVel = 0;
    }

    // set cw/ccw direction
    if ((targetAngle < 0 && rotVel > 0) || (targetAngle > 0 && rotVel < 0)) {
      rotVel *= -1;
    }
    return rotVel;
  }
}
