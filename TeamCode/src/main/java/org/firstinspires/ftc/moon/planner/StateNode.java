package org.firstinspires.ftc.moon.planner;

import java.util.ArrayList;
import java.util.List;

/**
 * State node for Hybrid A* search tree.
 * Corresponds to the C++ StateNode struct.
 */
@SuppressWarnings("ALL")
public class StateNode {
    
    /** Node status in the search */
    public enum NodeStatus {
        NOT_VISITED,
        IN_OPENSET,
        IN_CLOSESET
    }
    
    /** Driving direction */
    public enum Direction {
        FORWARD,
        BACKWARD,
        NO
    }
    
    // Grid index in the state lattice
    public final Vec3i gridIndex;
    
    // Continuous state (x, y, heading)
    public Vec3d state;
    
    // Node status
    public NodeStatus nodeStatus = NodeStatus.NOT_VISITED;
    
    // Driving direction
    public Direction direction = Direction.NO;
    
    // Cost values
    public double gCost;  // Cost from start to this node
    public double fCost;  // Total cost (g + h)
    
    // Steering grade (discretized steering angle index)
    public int steeringGrade;
    
    // Parent node for path backtracking
    public StateNode parentNode;
    
    // Intermediate states between parent and this node
    // (for smooth path reconstruction)
    public List<Vec3d> intermediateStates;
    
    /**
     * Create a state node at the given grid index.
     */
    public StateNode(Vec3i gridIndex) {
        this.gridIndex = new Vec3i(gridIndex);
        this.intermediateStates = new ArrayList<>();
    }
    
    /**
     * Get the position (x, y) from the state.
     */
    public Vec2d getPos() {
        return new Vec2d(state.getX(), state.getY());
    }

}