package org.firstinspires.ftc.moon.planner;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;
import java.util.List;

/**
 * Example demonstrating how to use HybridA* with FTC coordinates
 * and integrate with MoonRobot's path following system.
 * 
 * This shows the recommended usage pattern in OpModes.
 */
@SuppressWarnings("ALL")
public class HybridAStarDemo {
    
    /**
     * Recommended configuration for FTC competition.
     * Use this as a starting point and tune parameters for your robot.
     */
    public static class Config {
        // Vehicle parameters (in inches)
        public double steeringAngle = 15.0;          // degrees
        public int steeringAngleDiscreteNum = 1;     // steering steps
        public double wheelBase = 12.0;            // inches
        public double segmentLength = 10.0;        // inches
        public int segmentLengthDiscreteNum = 5;     // steps per segment
        
        // Cost penalties (tune these for your robot)
        public double steeringPenalty = 1.3;
        public double reversingPenalty = 2.5;
        public double steeringChangePenalty = 1.8;
        public double shotDistance = 12.0;         // inches
        
        // Grid settings (tune for speed vs accuracy)
        public double stateGridResolution = 2.0;   // inches per state cell
        public double mapGridResolution = 1.0;     // inches per obstacle cell
        public int headingGridDivisions = 72;
        
        // Robot dimensions (in inches)
        public double robotLength = 18.0;
        public double robotWidth = 18.0;
        public double rearAxleToRear = 6.0;
        
        // Path following constraints
        public double maxVel = 0.99;        // max velocity (0-1)
        public double maxAccel = 100;       // max acceleration
        public double maxAngularVel = 1.0; // max angular velocity
        public double maxAngularAccel = 1.5; // max angular acceleration
        
        public PathConstraints getConstraints() {
            return new PathConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel);
        }
        
        public HybridAStar createPlanner() {
            return new HybridAStar(
                steeringAngle, steeringAngleDiscreteNum,
                segmentLength, segmentLengthDiscreteNum,
                wheelBase,
                steeringPenalty, reversingPenalty, steeringChangePenalty,
                shotDistance,
                headingGridDivisions
            );
        }
    }
    
    /**
     * Main example showing FTC coordinate usage.
     */
    public static void main(String[] args) {
        Config config = new Config();
        
        // ============================================================
        // STEP 1: Create and configure the planner
        // ============================================================
        HybridAStar planner = config.createPlanner();
        
        // ============================================================
        // STEP 2: Initialize with FTC field dimensions
        // ============================================================
        double[] bounds = FTCConstants.getInternalBounds();
        
        planner.init(bounds[0], bounds[1], bounds[2], bounds[3], 
                     config.stateGridResolution, config.mapGridResolution);
        
        planner.setVehicleShape(config.robotLength, config.robotWidth, config.rearAxleToRear);
        
        // ============================================================
        // STEP 3: Add obstacles (FTC coordinates)
        // ============================================================
        // Example: obstacles at known positions
        planner.setObstacle(36.0, 36.0);   // Field center
        planner.setObstacle(100.0, 60.0);   // Red alliance side
        
        // ============================================================
        // STEP 4: Define start and goal (FTC coordinates)
        // ============================================================
        Vec3d startFTC = new Vec3d(10.0, 10.0, 0.0);        // Bottom-left, forward
        Vec3d goalFTC = new Vec3d(130.0, 130.0, Math.PI/2); // Top-right, facing right
        
        // ============================================================
        // STEP 5: Search!
        // ============================================================
        long startTime = System.currentTimeMillis();
        List<Vec3d> path = planner.search(startFTC, goalFTC);
        long endTime = System.currentTimeMillis();
        
        printResults("Search", path, planner.getPathLength(), endTime - startTime);
        
        if (path.isEmpty()) {
            System.out.println("NO PATH FOUND - check obstacle placement");
            return;
        }
        
        // ============================================================
        // STEP 6: Convert to PedroPath format
        // ============================================================
        List<Pose> pedroPath = PathConverter.toPoseList(path);
        
        System.out.println("\nPath in Pedro coordinates:");
        for (int i = 0; i < pedroPath.size(); i++) {
            Pose p = pedroPath.get(i);
            System.out.printf("  [%3d]: (%.1f, %.1f) heading=%.1f°\n",
                i, p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        }
        
        // ============================================================
        // STEP 7: Create PathChain for following
        // ============================================================
        PathChain chain = PathConverter.toPathChain(path, config.getConstraints());
        
        if (chain != null) {
            System.out.println("\nPathChain successfully created!");
            System.out.println("Ready for: moonRobot.followPath(chain)");
        } else {
            System.out.println("\nPathChain creation failed");
        }
    }
    
    /**
     * Integration example for Autonomous OpMode.
     * This is the recommended pattern for using HybridA* in competition.
     */
    public static class OpModeExample {
        
        private final Config config;
        private HybridAStar planner;
        private List<Vec3d> currentPath;
        private List<Pose> pedroPath;
        
        public OpModeExample(Config config) {
            this.config = config;
            this.planner = config.createPlanner();
            
            double[] bounds = FTCConstants.getInternalBounds();
            planner.init(bounds[0], bounds[1], bounds[2], bounds[3],
                         config.stateGridResolution, config.mapGridResolution);
            planner.setVehicleShape(config.robotLength, config.robotWidth, config.rearAxleToRear);
        }
        
        /**
         * Set the static obstacle map for the field.
         * Call this during opMode init.
         */
        public void setObstacleMap(List<Vec2d> obstacles) {
            planner.reset();
            for (Vec2d obs : obstacles) {
                planner.setObstacle(obs.x, obs.y);
            }
        }
        
        /**
         * Add a dynamic obstacle at runtime.
         */
        public void addObstacle(double x, double y) {
            planner.setObstacle(x, y);
        }
        
        /**
         * Plan a path from current position to goal.
         * Returns true if path was found.
         */
        public boolean plan(Vec3d startFTC, Vec3d goalFTC) {
            long startTime = System.currentTimeMillis();
            currentPath = planner.search(startFTC, goalFTC);
            long endTime = System.currentTimeMillis();
            
            if (currentPath.isEmpty()) {
                return false;
            }
            
            // Convert to Pedro format
            pedroPath = PathConverter.toPoseList(currentPath);
            
            System.out.println("[HybridA*] Path found in " + (endTime - startTime) + "ms");
            return true;
        }
        
        /**
         * Get the planned path as Pedro Poses.
         */
        public List<Pose> getPath() {
            return pedroPath;
        }
        
        /**
         * Get the raw path in FTC coordinates.
         */
        public List<Vec3d> getRawPath() {
            return currentPath;
        }
        
        /**
         * Get the search tree for visualization.
         */
        public List<double[]> getSearchTree() {
            return planner.getSearchedTree();
        }
        
        /**
         * Create a PathChain for the current path.
         */
        public PathChain getPathChain() {
            if (currentPath == null || currentPath.isEmpty()) {
                return null;
            }
            return PathConverter.toPathChain(currentPath, config.getConstraints());
        }
    }
    
    /**
     * Helper to print search results.
     */
    private static void printResults(String label, List<Vec3d> path, double length, long timeMs) {
        System.out.println("=== " + label + " Results ===");
        System.out.println("Time: " + timeMs + " ms");
        System.out.println("Length: " + String.format("%.1f", length) + " inches");
        System.out.println("Points: " + path.size());
        
        if (!path.isEmpty()) {
            Vec3d first = path.get(0);
            Vec3d last = path.get(path.size() - 1);
            System.out.println("From: (" + String.format("%.1f", first.x) + ", " + 
                             String.format("%.1f", first.y) + ")");
            System.out.println("To:   (" + String.format("%.1f", last.x) + ", " + 
                             String.format("%.1f", last.y) + ")");
        }
    }
}