package org.firstinspires.ftc.moon.planner;

/**
 * Top-level 2D vector used by planner classes.
 */
public class Vec2d {
    public double x;
    public double y;

    public Vec2d() {}

    public Vec2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vec2d(Vec2d other) {
        this.x = other.x;
        this.y = other.y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    public double normL1() {
        return Math.abs(x) + Math.abs(y);
    }

    public Vec2d plus(Vec2d other) {
        return new Vec2d(x + other.x, y + other.y);
    }

    public Vec2d minus(Vec2d other) {
        return new Vec2d(x - other.x, y - other.y);
    }

    public Vec2d scaled(double s) {
        return new Vec2d(x * s, y * s);
    }

    public Vec2d normalized() {
        double n = norm();
        return n > 1e-10 ? new Vec2d(x / n, y / n) : new Vec2d(0, 0);
    }

    @Override
    public String toString() {
        return String.format("(%.4f, %.4f)", x, y);
    }
}


