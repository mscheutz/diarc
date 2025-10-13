/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

// helper class to hold a 3D position and rotation
public class Pose {
  public final Vec3 position;
  public final Vec3 rotation; // Euler angles in radians

  public Pose(Vec3 position, Vec3 rotation) {
    this.position = position;
    this.rotation = rotation;
  }

  @Override
  public String toString() {
    return String.format("Position: %s, Rotation (radians): %s", position, rotation);
  }
}

