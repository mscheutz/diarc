/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing;

import java.awt.*;

/**
 * Represents the 2D quadrilateral projection of the box in the camera image.
 */
public class Quadrilateral {
    private final Vector2D p1, p2, p3, p4;

    /**
     * Constructs a quadrilateral from its four corner points.
     * The points should be ordered sequentially around the perimeter (e.g., clockwise).
     */
    public Quadrilateral(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4) {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
    }

    public Vector2D getP1() { return p1; }
    public Vector2D getP2() { return p2; }
    public Vector2D getP3() { return p3; }
    public Vector2D getP4() { return p4; }

    /**
     * @return The geometric center (centroid) of the quadrilateral's vertices.
     */
    public Vector2D getPixelCenter() {
        double centerX = (p1.x() + p2.x() + p3.x() + p4.x()) / 4.0;
        double centerY = (p1.y() + p2.y() + p3.y() + p4.y()) / 4.0;
        return new Vector2D(centerX, centerY);
    }

    public String toString() {
      StringBuilder builder = new StringBuilder();
      builder.append(p1);
      builder.append(", ");
      builder.append(p2);
      builder.append(", ");
      builder.append(p3);
      builder.append(", ");
      builder.append(p4);
      return builder.toString();
    }
}
