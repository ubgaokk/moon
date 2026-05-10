package org.firstinspires.ftc.moon.planner;

/**
 * Type definitions for Hybrid A* path planning.
 * Ports from C++ Eigen types to Java custom classes.
 * 
 * Note: Vec2d, Vec3d, Vec2i, Vec3i are defined as top-level classes in separate files.
 * This file only contains auxiliary types that don't have their own files.
 */
@SuppressWarnings("ALL")
public class Types {
    
    /** Path point wrapper: x, y, heading */
    public static class PathPoint {
        public final double x;
        public final double y;
        public final double heading;
        
        public PathPoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
        
        public PathPoint(Vec3d state) {
            this.x = state.x;
            this.y = state.y;
            this.heading = state.z;
        }
        
        public Vec3d toVec3d() {
            return new Vec3d(x, y, heading);
        }
        
        @Override
        public String toString() {
            return String.format("(%.2f, %.2f, %.2f°)", x, y, Math.toDegrees(heading));
        }
    }
}