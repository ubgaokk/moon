package org.firstinspires.ftc.moon.planner;

import java.util.Locale;

/**
 * Top-level 3D integer index for state lattice cells.
 */
public class Vec3i {
    public int x;
    public int y;
    public int z;

    public Vec3i(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3i(Vec3i other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    public String toString() {
        return String.format(Locale.US, "(%d, %d, %d)", x, y, z);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vec3i) {
            Vec3i other = (Vec3i) obj;
            return x == other.x && y == other.y && z == other.z;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
    }
}


