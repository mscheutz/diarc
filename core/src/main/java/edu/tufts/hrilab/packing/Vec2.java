package edu.tufts.hrilab.packing;

import java.util.List;

public class Vec2 {
    public final double x, y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vec2 add(Vec2 other) {
        return new Vec2(x + other.x, y + other.y);
    }

    public Vec2 subtract(Vec2 other) {
        return new Vec2(x - other.x, y - other.y);
    }

    public Vec2 scale(double factor) {
        return new Vec2(x * factor, y * factor);
    }

    public double magnitudeSq() {
        return x * x + y * y;
    }

    public double magnitude() {
        return Math.sqrt(magnitudeSq());
    }

    public double distanceSq(Vec2 other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return dx * dx + dy * dy;
    }

    public static Vec2 average(List<Vec2> vectors) {
        double sumX = 0, sumY = 0;
        for (Vec2 v : vectors) {
            sumX += v.x;
            sumY += v.y;
        }
        return new Vec2(sumX / vectors.size(), sumY / vectors.size());
    }
    
    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }
}