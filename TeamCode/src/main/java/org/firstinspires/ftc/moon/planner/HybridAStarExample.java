package org.firstinspires.ftc.moon.planner;

import java.util.List;

/**
 * Example/demo class showing how to use HybridAStar planner.
 * This demonstrates the basic usage pattern with FTC coordinates (inches).
 */
@SuppressWarnings("ALL")
public class HybridAStarExample {
    
    /**
     * Example usage of the Hybrid A* planner.
     */
    public static void main(String[] args) {
        // === Configuration ===
        // Vehicle parameters (in inches - FTC uses inches)
        double steeringAngle = 15.0;          // degrees
        int steeringAngleDiscreteNum = 1;     // Number of steering angle steps
        double wheelBase = 12.0;            // Distance between front and rear axles (inches)
        double segmentLength = 12.0;        // Length of each motion segment (inches)
        int segmentLengthDiscreteNum = 6;     // Number of steps per segment
        
        // Cost penalties
        double steeringPenalty = 1.3;       // Penalty for non-zero steering
        double reversingPenalty = 2.5;        // Penalty for reversing
        double steeringChangePenalty = 1.8;  // Penalty for changing steering
        double shotDistance = 10.0;         // Distance to use RS path shortcut (inches)
        
        // Create planner
        HybridAStar planner = new HybridAStar(
            steeringAngle, steeringAngleDiscreteNum,
            segmentLength, segmentLengthDiscreteNum,
            wheelBase,
            steeringPenalty, reversingPenalty, steeringChangePenalty,
            shotDistance,
            72  // grid size for phi (heading)
        );
        
        // === Map Setup ===
        // Map boundaries (FTC field: 144in x 144in)
        double xLower = 0, xUpper = 144.0;
        double yLower = 0, yUpper = 144.0;
        
        // Grid resolutions (in inches)
        double stateGridResolution = 2.0;   // For state lattice
        double mapGridResolution = 1.0;    // For occupancy grid
        
        // Initialize planner
        planner.init(xLower, xUpper, yLower, yUpper, stateGridResolution, mapGridResolution);
        
        // Set vehicle shape (length, width, rear axle to rear) in inches
        planner.setVehicleShape(18.0, 18.0, 6.0);  // 18x18 inch robot with axle 6" from rear
        
        // === Add Obstacles ===
        // Add some obstacles at FTC positions
        Vec3d obs1 = FTCConstants.ftcToInternal(36.0, 36.0, 0);
        Vec3d obs2 = FTCConstants.ftcToInternal(72.0, 50.0, 0);
        Vec3d obs3 = FTCConstants.ftcToInternal(100.0, 80.0, 0);
        planner.setObstacle(obs1.x, obs1.y);
        planner.setObstacle(obs2.x, obs2.y);
        planner.setObstacle(obs3.x, obs3.y);
        
        // === Define Start and Goal ===
        // Using FTC coordinates (x, y, heading in radians)
        // Heading: 0 = forward (+Y), PI/2 = right (+X), PI = backward (-Y)
        Vec3d startFTC = new Vec3d(10.0, 10.0, 0.0);                    // Bottom-left corner, facing forward
        Vec3d goalFTC = new Vec3d(130.0, 130.0, Math.toRadians(90));   // Top-right area, facing right
        Vec3d start = FTCConstants.ftcToInternal(startFTC.x, startFTC.y, startFTC.z);
        Vec3d goal = FTCConstants.ftcToInternal(goalFTC.x, goalFTC.y, goalFTC.z);
        
        // === Search ===
        long startTime = System.currentTimeMillis();
        List<Vec3d> path = planner.search(start, goal);
        long endTime = System.currentTimeMillis();
        
        // === Results ===
        System.out.println("=== Hybrid A* Search Results ===");
        System.out.println("Search time: " + (endTime - startTime) + " ms");
        System.out.println("Path length: " + planner.getPathLength() + " inches");
        System.out.println("Path points: " + path.size());

        if (path.isEmpty()) {
            System.out.println("No path found!");
            return;
        }

        System.out.println("\nPath (FTC coordinates):");
        for (int i = 0; i < path.size(); i++) {
            Vec3d pt = path.get(i);
            Vec3d ftcPt = FTCConstants.internalToFTC(pt.x, pt.y, pt.z);
            System.out.printf("  [%3d]: (%.1f, %.1f) heading=%.1f°\n", 
                i, ftcPt.x, ftcPt.y, Math.toDegrees(ftcPt.z));
        }
        
        // === Get Search Tree (for visualization) ===
        List<double[]> tree = planner.getSearchedTree();
        System.out.println("\nSearch tree has " + tree.size() + " edges");
        
        // === Convert to Pedro coordinates for MoonRobot ===
        List<com.pedropathing.geometry.Pose> pedroPath = PathConverter.toPoseList(path);
        System.out.println("\nPath in Pedro coordinates:");
        for (int i = 0; i < pedroPath.size(); i++) {
            com.pedropathing.geometry.Pose p = pedroPath.get(i);
            System.out.printf("  [%3d]: (%.1f, %.1f) heading=%.1f°\n",
                i, p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        }
    }
}