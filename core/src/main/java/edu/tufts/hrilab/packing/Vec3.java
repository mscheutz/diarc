/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

  // simple 3D vector class
public class Vec3 {
    public final double x, y, z;

    public Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }

    public edu.tufts.hrilab.packing.Vec3 toDegrees() {
      return new edu.tufts.hrilab.packing.Vec3(Math.toDegrees(x), Math.toDegrees(y), Math.toDegrees(z));
    }

    @Override
    public String toString() {
      return String.format("(%.3f, %.3f, %.3f)", x, y, z);
    }
}
