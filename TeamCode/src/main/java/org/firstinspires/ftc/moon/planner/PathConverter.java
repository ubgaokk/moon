package org.firstinspires.ftc.moon.planner;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.geometry.BezierCurve;

import java.util.ArrayList;
import java.util.List;

/**
 * Convert HybridA* path results to PedroPath format.
 * PedroPath 2.0.4 API: Path(BezierCurve), PathChain(Path...), BezierCurve(List<Pose>)
 */
@SuppressWarnings("ALL")
public class PathConverter {
    
    /**
     * Convert path points to list of Pedro Poses.
     * @param pathPoints HybridA* output (internal coords)
     * @return Poses in Pedro coords
     */
    public static List<Pose> toPoseList(List<Vec3d> pathPoints) {
        if (pathPoints == null || pathPoints.isEmpty()) {
            return new ArrayList<>();
        }
        
        List<Pose> poses = new ArrayList<>(pathPoints.size());
        for (Vec3d pt : pathPoints) {
            Vec3d ftc = FTCConstants.internalToFTC(pt.x, pt.y, pt.z);
            poses.add(FTCConstants.ftcToPedro(ftc.x, ftc.y, ftc.z));
        }
        return poses;
    }
    
    /**
     * Convert to Path (single BezierCurve).
     * @param pathPoints HybridA* output
     * @param constraints path constraints (null = default)
     * @return Path or null
     */
    public static Path toPath(List<Vec3d> pathPoints, PathConstraints constraints) {
        if (pathPoints == null || pathPoints.isEmpty()) {
            return null;
        }
        
        List<Pose> poses = toPoseList(pathPoints);
        if (poses.isEmpty()) {
            return null;
        }
        
        try {
            BezierCurve curve = new BezierCurve(poses);
            return (constraints != null) ? new Path(curve, constraints) : new Path(curve);
        } catch (Exception e) {
            System.err.println("Path build failed: " + e.getMessage());
            return null;
        }
    }
    
    /**
     * Convert to Path with default constraints.
     */
    public static Path toPath(List<Vec3d> pathPoints) {
        return toPath(pathPoints, null);
    }
    
    /**
     * Convert to PathChain (single Path wrapped in Chain).
     * @param pathPoints HybridA* output
     * @param constraints path constraints (null = default)
     * @return PathChain or null
     */
    public static PathChain toPathChain(List<Vec3d> pathPoints, PathConstraints constraints) {
        if (pathPoints == null || pathPoints.isEmpty()) {
            return null;
        }
        
        List<Pose> poses = toPoseList(pathPoints);
        if (poses.size() < 2) {
            return null;
        }
        
        try {
            BezierCurve curve = new BezierCurve(poses);
            Path path = (constraints != null) ? new Path(curve, constraints) : new Path(curve);
            return new PathChain(path);
        } catch (Exception e) {
            System.err.println("PathChain build failed: " + e.getMessage());
            return null;
        }
    }
    
    /**
     * Convert to PathChain with default constraints.
     */
    public static PathChain toPathChain(List<Vec3d> pathPoints) {
        return toPathChain(pathPoints, null);
    }
    
    /**
     * Create interpolated path between two waypoints.
     * @param start start (internal coords)
     * @param end end (internal coords)
     * @param points number of interpolated points
     * @return list of poses in Pedro coords
     */
    public static List<Pose> createLinearPath(Vec3d start, Vec3d end, int points) {
        List<Pose> poses = new ArrayList<>(points + 1);
        
        Vec3d s = FTCConstants.internalToFTC(start.x, start.y, start.z);
        Vec3d e = FTCConstants.internalToFTC(end.x, end.y, end.z);
        
        for (int i = 0; i <= points; i++) {
            double t = (double) i / points;
            double h = s.z + t * normalizeAngle(e.z - s.z);
            poses.add(FTCConstants.ftcToPedro(
                s.x + t * (e.x - s.x),
                s.y + t * (e.y - s.y),
                h
            ));
        }
        return poses;
    }
    
    /**
     * Create smooth path through waypoints (linear interpolation).
     * @param waypoints list (internal coords)
     * @param pointsPerSegment interpolated points between each pair
     * @return poses in Pedro coords
     */
    public static List<Pose> createSmoothPath(List<Vec3d> waypoints, int pointsPerSegment) {
        if (waypoints == null || waypoints.isEmpty()) {
            return new ArrayList<>();
        }
        
        List<Pose> result = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            List<Pose> seg = createLinearPath(waypoints.get(i), waypoints.get(i + 1), pointsPerSegment);
            result.addAll(seg.subList(0, seg.size() - 1)); // exclude last (next segment's start)
        }
        
        // Add final point
        Vec3d last = waypoints.get(waypoints.size() - 1);
        Vec3d l = FTCConstants.internalToFTC(last.x, last.y, last.z);
        result.add(FTCConstants.ftcToPedro(l.x, l.y, l.z));
        
        return result;
    }
    
    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}