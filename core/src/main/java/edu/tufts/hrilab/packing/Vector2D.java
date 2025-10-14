/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

/**
 * A simple record to represent a 2D vector or point with double precision.
 */
public record Vector2D(double x, double y) {
    @Override
    public String toString() {
        return String.format("(%.4f, %.4f)", x, y);
    }

    public double distanceTo(Vector2D other) {
        return Math.sqrt(Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2));
    }
}
