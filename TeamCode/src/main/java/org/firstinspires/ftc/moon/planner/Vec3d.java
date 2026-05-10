package org.firstinspires.ftc.moon.planner;

/**
 * Top-level 3D planner state (x, y, heading).
 */
public class Vec3d {
    public double x;
    public double y;
    public double z;

    public Vec3d() {}

    public Vec3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3d(Vec3d other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
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

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    public double normL1() {
        return Math.abs(x) + Math.abs(y);
    }

    public Vec2d getPos() {
        return new Vec2d(x, y);
    }

    public double distanceTo(Vec3d other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public String toString() {
        return String.format("(%.4f, %.4f, %.4f)", x, y, z);
    }
}


