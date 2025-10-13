package edu.tufts.hrilab.packing;

import java.util.List;

public class Vec3 {
    public final double x, y, z;

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
	this.z = z;
    }

    public Vec3 subtract(Vec3 other) {
        return new Vec3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public static Vec3 average(List<Vec3> vectors) {
        if (vectors == null || vectors.isEmpty()) {
            return new Vec3(0, 0, 0);
        }
        double sumX = 0, sumY = 0, sumZ = 0;
        for (Vec3 vec : vectors) {
            sumX += vec.x;
            sumY += vec.y;
	    sumZ += vec.z;
        }
        return new Vec3(sumX / vectors.size(), sumY / vectors.size(), sumZ / vectors.size());
    }

    public double getX() {
      return x;
    }

    public double getY() {
      return y;
    }

    public double getZ() {
      return z;
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.2f)", x, y, z);
    }
}
