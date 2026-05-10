package org.firstinspires.ftc.moon.planner;

import java.util.*;

/**
 * Hybrid A* path planner for FTC robots.
 * Ports from the C++ HybridAStar implementation with modifications for Java.
 * 
 * Features:
 * - Kinodynamic search with car-like vehicle model
 * - Reed-Shepp path for analytic expansion
 * - Obstacle collision detection with vehicle shape
 * - Support for forward and backward driving
 */
@SuppressWarnings("ALL")
public class HybridAStar {
    
    // Map boundaries
    private double mapXLower, mapXUpper;
    private double mapYLower, mapYUpper;
    
    // Grid resolutions
    private double stateGridResolution;  // For state lattice
    private double mapGridResolution;    // For occupancy grid
    private double angularResolution;
    
    // State grid sizes
    private int stateGridSizeX, stateGridSizeY, stateGridSizePhi;
    private int mapGridSizeX, mapGridSizeY;
    
    // Vehicle parameters
    private double wheelBase;
    private double segmentLength;
    private double moveStepSize;
    private double steeringRadian;
    private double steeringRadianStepSize;
    private int steeringDiscreteNum;
    private int segmentLengthDiscreteNum;
    
    // Cost penalties
    private double steeringPenalty;
    private double reversingPenalty;
    private double steeringChangePenalty;
    private double shotDistance;
    
    // Search data
    private byte[] mapData;  // Occupancy grid: 0=free, 1=occupied
    private StateNode[][][] stateNodeMap;  // 3D state lattice
    
    // Open set (priority queue) - using TreeSet with comparator
    private TreeSet<StateNode> openSet;
    
    // Terminal node (goal)
    private StateNode terminalNode;
    
    // Path length
    private double pathLength;
    
    // RS Path planner
    private RSPath rsPath;
    
    // Vehicle shape (for collision detection)
    private Vec2d[] vehicleShape;       // 4 corners (8 points)
    private int vehiclePointsCount;
    
    // Constants
    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double EPS = 1e-6;
    
    /**
     * Create a Hybrid A* planner with given parameters.
     */
    public HybridAStar(double steeringAngle,       // degrees
                       int steeringAngleDiscreteNum,
                       double segmentLength,        // meters
                       int segmentLengthDiscreteNum,
                       double wheelBase,            // meters
                       double steeringPenalty,
                       double reversingPenalty,
                       double steeringChangePenalty,
                       double shotDistance,
                       int gridSizePhi) {
        
        this.wheelBase = wheelBase;
        this.segmentLength = segmentLength;
        this.steeringPenalty = steeringPenalty;
        this.reversingPenalty = reversingPenalty;
        this.steeringChangePenalty = steeringChangePenalty;
        this.shotDistance = shotDistance;
        
        // Steering angle discretization
        this.steeringRadian = Math.toRadians(steeringAngle);
        this.steeringDiscreteNum = steeringAngleDiscreteNum;
        this.steeringRadianStepSize = steeringRadian / steeringDiscreteNum;
        
        // Segment discretization
        this.segmentLengthDiscreteNum = segmentLengthDiscreteNum;
        this.moveStepSize = segmentLength / segmentLengthDiscreteNum;
        
        // RS Path
        double turningRadius = wheelBase / Math.tan(steeringRadian);
        this.rsPath = new RSPath(turningRadius);
        
        // Phi discretization
        this.stateGridSizePhi = gridSizePhi;
        this.angularResolution = TWO_PI / gridSizePhi;
        
        this.pathLength = 0.0;
    }
    
    /**
     * Initialize the planner with map boundaries and resolutions.
     */
    public void init(double xLower, double xUpper, 
                     double yLower, double yUpper,
                     double stateGridResolution,
                     double mapGridResolution) {
        
        this.mapXLower = xLower;
        this.mapXUpper = xUpper;
        this.mapYLower = yLower;
        this.mapYUpper = yUpper;
        this.stateGridResolution = stateGridResolution;
        this.mapGridResolution = mapGridResolution;
        
        // State grid sizes
        this.stateGridSizeX = (int) Math.floor((mapXUpper - mapXLower) / stateGridResolution);
        this.stateGridSizeY = (int) Math.floor((mapYUpper - mapYLower) / stateGridResolution);
        
        // Map grid sizes
        this.mapGridSizeX = (int) Math.floor((mapXUpper - mapXLower) / mapGridResolution);
        this.mapGridSizeY = (int) Math.floor((mapYUpper - mapYLower) / mapGridResolution);
        
        // Initialize map data
        this.mapData = new byte[mapGridSizeX * mapGridSizeY];
        Arrays.fill(mapData, (byte) 0);
        
        // Initialize state node map
        this.stateNodeMap = new StateNode[stateGridSizeX][stateGridSizeY][stateGridSizePhi];
        
        // Open set
        this.openSet = new TreeSet<>((a, b) -> {
            if (Math.abs(a.fCost - b.fCost) < 1e-6) {
                return a.hashCode() - b.hashCode();
            }
            return Double.compare(a.fCost, b.fCost);
        });
        
        // Vehicle shape
        setVehicleShape(0.99, 0.576, 0.576);
    }
    
    /**
     * Set vehicle shape for collision detection.
     */
    public void setVehicleShape(double length, double width, double rearAxleToRear) {
        vehicleShape = new Vec2d[4];
        vehicleShape[0] = new Vec2d(-rearAxleToRear, width / 2);
        vehicleShape[1] = new Vec2d(length - rearAxleToRear, width / 2);
        vehicleShape[2] = new Vec2d(length - rearAxleToRear, -width / 2);
        vehicleShape[3] = new Vec2d(-rearAxleToRear, -width / 2);
        vehiclePointsCount = 4;
    }
    
    /**
     * Set an obstacle at the given grid position.
     */
    public void setObstacle(int gridX, int gridY) {
        if (gridX >= 0 && gridX < mapGridSizeX && gridY >= 0 && gridY < mapGridSizeY) {
            mapData[gridX + gridY * mapGridSizeX] = 1;
        }
    }
    
    /**
     * Set an obstacle at the given world position.
     */
    public void setObstacle(double worldX, double worldY) {
        if (worldX < mapXLower || worldX > mapXUpper || 
            worldY < mapYLower || worldY > mapYUpper) {
            return;
        }
        
        int gridX = (int) ((worldX - mapXLower) / mapGridResolution);
        int gridY = (int) ((worldY - mapYLower) / mapGridResolution);
        
        setObstacle(gridX, gridY);
    }
    
    /**
     * Set obstacles from a 2D occupancy grid (0=free, non-zero=occupied).
     * The grid is scaled to match mapGridResolution.
     */
    public void setObstacleGrid(byte[][] grid, double resolution) {
        int gridW = grid[0].length;
        int gridH = grid.length;
        
        int mapW = (int) Math.floor((mapXUpper - mapXLower) / mapGridResolution);
        int mapH = (int) Math.floor((mapYUpper - mapYLower) / mapGridResolution);
        
        for (int my = 0; my < mapH; my++) {
            for (int mx = 0; mx < mapW; mx++) {
                double worldX = mapXLower + mx * mapGridResolution;
                double worldY = mapYLower + my * mapGridResolution;
                
                int gridX = (int) (worldX / resolution);
                int gridY = (int) (worldY / resolution);
                
                if (gridX >= 0 && gridX < gridW && gridY >= 0 && gridY < gridH) {
                    if (grid[gridY][gridX] != 0) {
                        setObstacle(mx, my);
                    }
                }
            }
        }
    }
    
    /**
     * Main search function - find a path from start to goal.
     * @return List of path points, or empty list if no path found
     */
    public List<Vec3d> search(Vec3d startState, Vec3d goalState) {
        // Get grid indices
        Vec3i startIndex = stateToIndex(startState);
        Vec3i goalIndex = stateToIndex(goalState);
        
        // Create goal node
        terminalNode = new StateNode(goalIndex);
        terminalNode.state = new Vec3d(goalState.x, goalState.y, goalState.z);
        terminalNode.direction = StateNode.Direction.NO;
        terminalNode.steeringGrade = 0;
        
        // Create start node
        StateNode startNode = new StateNode(startIndex);
        startNode.state = new Vec3d(startState.x, startState.y, startState.z);
        startNode.steeringGrade = 0;
        startNode.direction = StateNode.Direction.NO;
        startNode.nodeStatus = StateNode.NodeStatus.IN_OPENSET;
        startNode.gCost = 0;
        startNode.fCost = computeH(startNode, terminalNode);
        startNode.intermediateStates = new ArrayList<>();
        startNode.intermediateStates.add(new Vec3d(startState.x, startState.y, startState.z));
        
        // Add to maps
        stateNodeMap[startIndex.x][startIndex.y][startIndex.z] = startNode;
        stateNodeMap[goalIndex.x][goalIndex.y][goalIndex.z] = terminalNode;
        
        // Add to open set
        openSet.add(startNode);
        
        // Search loop
        int iterations = 0;
        int maxIterations = 50000;
        
        while (!openSet.isEmpty() && iterations < maxIterations) {
            // Pop node with lowest f cost
            StateNode currentNode = openSet.pollFirst();
            currentNode.nodeStatus = StateNode.NodeStatus.IN_CLOSESET;
            
            // Check if we've reached the goal
            Vec2d currentPos = new Vec2d(currentNode.state.x, currentNode.state.y);
            Vec2d goalPos = new Vec2d(goalState.x, goalState.y);
            
            if (currentPos.minus(goalPos).norm() <= shotDistance) {
                // Try analytic expansion with RS path
                double rsLength = 0;
                if (analyticExpansion(currentNode, goalState, terminalNode)) {
                    // Success - reconstruct path
                    return reconstructPath();
                }
            }
            
            // Get neighbors
            List<StateNode> neighbors = getNeighborNodes(currentNode);
            
            for (StateNode neighbor : neighbors) {
                // Compute edge cost
                double edgeCost = computeG(currentNode, neighbor);
                
                // Compute heuristic
                double h = computeH(neighbor, terminalNode) * 1.001;  // tie-breaker
                
                Vec3i index = neighbor.gridIndex;
                
                // If not visited
                StateNode existing = stateNodeMap[index.x][index.y][index.z];
                if (existing == null) {
                    neighbor.gCost = currentNode.gCost + edgeCost;
                    neighbor.fCost = neighbor.gCost + h;
                    neighbor.parentNode = currentNode;
                    neighbor.nodeStatus = StateNode.NodeStatus.IN_OPENSET;
                    
                    stateNodeMap[index.x][index.y][index.z] = neighbor;
                    openSet.add(neighbor);
                }
                // If in open set, check if we found a better path
                else if (existing.nodeStatus == StateNode.NodeStatus.IN_OPENSET) {
                    double newGCost = currentNode.gCost + edgeCost;
                    if (existing.gCost > newGCost) {
                        existing.gCost = newGCost;
                        existing.fCost = newGCost + h;
                        existing.parentNode = currentNode;
                        
                        // Remove and re-add to update position in tree
                        openSet.remove(existing);
                        openSet.add(existing);
                    }
                }
            }
            
            iterations++;
        }
        
        // No path found
        return new ArrayList<>();
    }
    
    /**
     * Get neighbor nodes by expanding in all steering directions.
     */
    private List<StateNode> getNeighborNodes(StateNode node) {
        List<StateNode> neighbors = new ArrayList<>();
        
        for (int i = -steeringDiscreteNum; i <= steeringDiscreteNum; i++) {
            // Forward expansion
            List<Vec3d> forwardStates = expandStep(node, i, true);
            if (!forwardStates.isEmpty()) {
                Vec3d endForward = forwardStates.get(forwardStates.size() - 1);
                if (!beyondBoundary(new Vec2d(endForward.x, endForward.y))) {
                if (checkCollision(forwardStates)) {
                    Vec3d finalState = forwardStates.get(forwardStates.size() - 1);
                    Vec3i gridIndex = stateToIndex(finalState);
                    
                    StateNode forwardNode = new StateNode(gridIndex);
                    forwardNode.state = new Vec3d(finalState.x, finalState.y, finalState.z);
                    forwardNode.intermediateStates = new ArrayList<>(forwardStates);
                    forwardNode.steeringGrade = i;
                    forwardNode.direction = StateNode.Direction.FORWARD;
                    neighbors.add(forwardNode);
                }
                }
            }
            
            // Backward expansion
            List<Vec3d> backwardStates = expandStep(node, i, false);
            if (!backwardStates.isEmpty()) {
                Vec3d endBackward = backwardStates.get(backwardStates.size() - 1);
                if (!beyondBoundary(new Vec2d(endBackward.x, endBackward.y))) {
                if (checkCollision(backwardStates)) {
                    Vec3d finalState = backwardStates.get(backwardStates.size() - 1);
                    Vec3i gridIndex = stateToIndex(finalState);
                    
                    StateNode backwardNode = new StateNode(gridIndex);
                    backwardNode.state = new Vec3d(finalState.x, finalState.y, finalState.z);
                    backwardNode.intermediateStates = new ArrayList<>(backwardStates);
                    backwardNode.steeringGrade = i;
                    backwardNode.direction = StateNode.Direction.BACKWARD;
                    neighbors.add(backwardNode);
                }
                }
            }
        }
        
        return neighbors;
    }
    
    /**
     * Expand from current node in a given steering direction.
     */
    private List<Vec3d> expandStep(StateNode node, int steeringIdx, boolean forward) {
        List<Vec3d> states = new ArrayList<>();
        
        double phi = steeringIdx * steeringRadianStepSize;
        double stepSign = forward ? 1.0 : -1.0;
        
        double x = node.state.x;
        double y = node.state.y;
        double theta = node.state.z;
        
        for (int i = 0; i < segmentLengthDiscreteNum; i++) {
            // Bicycle model kinematic update
            double step = stepSign * moveStepSize;
            x += step * Math.cos(theta);
            y += step * Math.sin(theta);
            theta = mod2Pi(theta + step / wheelBase * Math.tan(phi));
            
            states.add(new Vec3d(x, y, theta));
        }
        
        return states;
    }
    
    /**
     * Compute heuristic cost (h) for a node.
     */
    private double computeH(StateNode node, StateNode goal) {
        // L1 norm (faster than L2)
        double h = node.state.getPos().minus(goal.state.getPos()).normL1();
        
        // If close to goal, use RS distance
        double dist = node.state.distanceTo(goal.state);
        if (dist < 3.0 * shotDistance) {
            h = rsPath.distance(
                node.state.x, node.state.y, node.state.z,
                goal.state.x, goal.state.y, goal.state.z
            );
        }
        
        return h;
    }
    
    /**
     * Compute edge cost (g) for moving from current to neighbor.
     */
    private double computeG(StateNode current, StateNode neighbor) {
        double g;
        
        if (neighbor.direction == StateNode.Direction.FORWARD) {
            if (neighbor.steeringGrade != current.steeringGrade) {
                if (neighbor.steeringGrade == 0) {
                    g = segmentLength * steeringChangePenalty;
                } else {
                    g = segmentLength * steeringChangePenalty * steeringPenalty;
                }
            } else {
                if (neighbor.steeringGrade == 0) {
                    g = segmentLength;
                } else {
                    g = segmentLength * steeringPenalty;
                }
            }
        } else {  // BACKWARD
            if (neighbor.steeringGrade != current.steeringGrade) {
                if (neighbor.steeringGrade == 0) {
                    g = segmentLength * steeringChangePenalty * reversingPenalty;
                } else {
                    g = segmentLength * steeringChangePenalty * steeringPenalty * reversingPenalty;
                }
            } else {
                if (neighbor.steeringGrade == 0) {
                    g = segmentLength * reversingPenalty;
                } else {
                    g = segmentLength * steeringPenalty * reversingPenalty;
                }
            }
        }
        
        return g;
    }
    
    /**
     * Try to connect current node to goal with RS path.
     */
    private boolean analyticExpansion(StateNode current, Vec3d goal, StateNode goalNode) {
        double step = moveStepSize;
        
        List<Vec3d> rsPathPoints = rsPath.getPath(current.state, new Vec3d(goal.x, goal.y, goal.z), step);
        
        // Check if path is valid
        for (Vec3d pt : rsPathPoints) {
            if (beyondBoundary(new Vec2d(pt.x, pt.y))) {
                return false;
            }
            if (!checkSingleCollision(pt.x, pt.y, pt.z)) {
                return false;
            }
        }
        
        // Success - set the path
        goalNode.intermediateStates = rsPathPoints;
        goalNode.parentNode = current;
        
        // Remove duplicate start point
        if (!goalNode.intermediateStates.isEmpty()) {
            goalNode.intermediateStates.remove(0);
        }
        
        return true;
    }
    
    /**
     * Reconstruct path from terminal to start.
     */
    private List<Vec3d> reconstructPath() {
        List<Vec3d> path = new ArrayList<>();
        
        List<StateNode> nodes = new ArrayList<>();
        StateNode node = terminalNode;
        
        while (node != null) {
            nodes.add(node);
            node = node.parentNode;
        }
        
        Collections.reverse(nodes);
        
        for (StateNode n : nodes) {
            path.addAll(n.intermediateStates);
        }
        
        return path;
    }
    
    /**
     * Convert world state to grid index.
     */
    private Vec3i stateToIndex(Vec3d state) {
        int x = (int) Math.max(0, Math.min(stateGridSizeX - 1, 
            (state.x - mapXLower) / stateGridResolution));
        int y = (int) Math.max(0, Math.min(stateGridSizeY - 1,
            (state.y - mapYLower) / stateGridResolution));
        int z = (int) Math.max(0, Math.min(stateGridSizePhi - 1,
            (state.z + Math.PI) / angularResolution));
        
        return new Vec3i(x, y, z);
    }
    
    /**
     * Check if position is beyond map boundaries.
     */
    private boolean beyondBoundary(Vec2d pos) {
        return pos.x < mapXLower || pos.x > mapXUpper ||
               pos.y < mapYLower || pos.y > mapYUpper;
    }
    
    /**
     * Check collision for a list of states.
     */
    private boolean checkCollision(List<Vec3d> states) {
        for (Vec3d state : states) {
            if (!checkSingleCollision(state.x, state.y, state.z)) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * Check collision for a single state with vehicle shape.
     */
    private boolean checkSingleCollision(double x, double y, double theta) {
        // Transform vehicle corners
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);
        
        Vec2d[] transformed = new Vec2d[4];
        for (int i = 0; i < 4; i++) {
            double cx = vehicleShape[i].x * cosT - vehicleShape[i].y * sinT + x;
            double cy = vehicleShape[i].x * sinT + vehicleShape[i].y * cosT + y;
            transformed[i] = new Vec2d(cx, cy);
        }
        
        // Check if any corner is in obstacle
        for (Vec2d pt : transformed) {
            int gridX = (int) ((pt.x - mapXLower) / mapGridResolution);
            int gridY = (int) ((pt.y - mapYLower) / mapGridResolution);
            
            if (gridX < 0 || gridX >= mapGridSizeX || gridY < 0 || gridY >= mapGridSizeY) {
                return false;  // Out of bounds = collision
            }
            
            if (mapData[gridX + gridY * mapGridSizeX] == 1) {
                return false;
            }
        }
        
        // Check vehicle edges using Bresenham-style line check
        for (int i = 0; i < 4; i++) {
            Vec2d p1 = transformed[i];
            Vec2d p2 = transformed[(i + 1) % 4];
            
            if (!lineCheck(p1, p2)) {
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * Check if a line segment intersects any obstacles.
     */
    private boolean lineCheck(Vec2d p1, Vec2d p2) {
        // Bresenham-style line check
        double x0 = p1.x, y0 = p1.y, x1 = p2.x, y1 = p2.y;
        
        boolean steep = Math.abs(y1 - y0) > Math.abs(x1 - x0);
        
        if (steep) {
            double tmp = x0;
            x0 = y0;
            y0 = tmp;
            tmp = x1;
            x1 = y1;
            y1 = tmp;
        }
        
        if (x0 > x1) {
            double tmp = x0;
            x0 = x1;
            x1 = tmp;
            tmp = y0;
            y0 = y1;
            y1 = tmp;
        }
        
        double deltaX = x1 - x0;
        double deltaY = Math.abs(y1 - y0);
        double error = deltaY / deltaX;
        
        double yStep = y0 < y1 ? 1 : -1;
        double y = y0;
        
        int N = (int) Math.abs(x1 - x0);
        for (int i = 0; i < N; i++) {
            double checkX = steep ? y : x0 + i * 1.0;
            double checkY = steep ? x0 + i * 1.0 : y;
            
            int gridX = (int) ((checkX - mapXLower) / mapGridResolution);
            int gridY = (int) ((checkY - mapYLower) / mapGridResolution);
            
            if (gridX < 0 || gridX >= mapGridSizeX || gridY < 0 || gridY >= mapGridSizeY) {
                return false;
            }
            
            if (mapData[gridX + gridY * mapGridSizeX] == 1) {
                return false;
            }
            
            error += deltaY;
            if (error >= 0.5) {
                y += yStep;
                error -= 1.0;
            }
        }
        
        return true;
    }
    
    /**
     * Normalize angle to [-PI, PI].
     */
    private static double mod2Pi(double angle) {
        while (angle > Math.PI) angle -= TWO_PI;
        while (angle < -Math.PI) angle += TWO_PI;
        return angle;
    }
    
    /**
     * Reset the planner for a new search.
     */
    public void reset() {
        if (stateNodeMap != null) {
            for (int i = 0; i < stateGridSizeX; i++) {
                for (int j = 0; j < stateGridSizeY; j++) {
                    for (int k = 0; k < stateGridSizePhi; k++) {
                        stateNodeMap[i][j][k] = null;
                    }
                }
            }
        }
        
        pathLength = 0.0;
        terminalNode = null;
        openSet.clear();
    }
    
    /**
     * Get path length from last successful search.
     */
    public double getPathLength() {
        return pathLength;
    }
    
    /**
     * Get the searched tree for visualization.
     * Returns a list of point pairs [x1, y1, x2, y2].
     */
    public List<double[]> getSearchedTree() {
        List<double[]> tree = new ArrayList<>();
        
        for (int i = 0; i < stateGridSizeX; i++) {
            for (int j = 0; j < stateGridSizeY; j++) {
                for (int k = 0; k < stateGridSizePhi; k++) {
                    StateNode node = stateNodeMap[i][j][k];
                    if (node != null && node.parentNode != null) {
                        List<Vec3d> states = node.intermediateStates;
                        for (int l = 0; l < states.size() - 1; l++) {
                            tree.add(new double[]{
                                states.get(l).x, states.get(l).y,
                                states.get(l + 1).x, states.get(l + 1).y
                            });
                        }
                    }
                }
            }
        }
        
        return tree;
    }
}