/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;

/**
 * From Ava documentation:
 *
 * position.x: the x coordinate of the center of the obstacle, in map coordinates.
 * position.y: the y coordinate of the center of the obstacle, in map coordinates.
 * position.theta: orientation of the bounding box around the obstacle, in radians, increasing in the counterclockwise direction.
 * velocity.x the x-dot velocity of the obstacle, in map coordinates
 * velocity.y the y-dot velocity of the obstacle, in map coordinates
 * velocity.theta: orientation of the velocity of the obstacle, in the obstacle's local coordinates
 * size.x: half the size along the obstacle's local x axis (forward).
 * size.y: half the size along the obstacle's local y axis (right).
 * type: the classification of the obstacle's relative position and motion, see the following table for interpretation of this integer value.
 *       0 IN_QUADRANT
 *       1 PASSING_SIDE
 *       2 PASSING_FRONT
 *       3 PASSING_BEHIND
 *       4 CROSSING
 *       5 APPROACHING
 */
public class Obstacle {
      public Position position;
      public Velocity velocity;
      public Size size;
      public String type;
}
