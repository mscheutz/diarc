/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025.
 */

package edu.tufts.hrilab.packing;

/**
 * Represents the final required adjustment for the robot arm, including translation and rotation.
 */
public record RobotAdjustment(Vector2D offset, double rotationDegrees) {
    @Override
    public String toString() {
        return String.format("Offset: %s, Rotation: %.2f degrees", offset, rotationDegrees);
    }

    public double getX() {
      return offset.x();
    }

    public double getY() {
      return offset.y();
    }

    public double getRotation() {
      return rotationDegrees * (Math.PI / 180);
    }
}
