package org.firstinspires.ftc.moon.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * AprilTag Detection Block using FTC Vision Portal
 * 
 * Wraps the standard FTC AprilTag detection API for easy use in autonomous programs.
 * Works with any camera (webcam, LL, etc.) configured for AprilTag detection.
 * 
 * Usage:
 *   AprilTagDetectionBlock aprilTag = new AprilTagDetectionBlock(hardwareMap, telemetry);
 *   // In loop:
 *   aprilTag.update();
 *   AprilTagDetection closest = aprilTag.getClosestTag();
 */
@SuppressWarnings("ALL")
public class AprilTagDetectionBlock {
    
    private final AprilTagProcessor aprilTagProcessor;
    private final Telemetry telemetry;
    private List<AprilTagDetection> currentDetections;
    private AprilTagDetection closestTag;
    
    // FTC field constants (144" x 144")
    public static final double FIELD_WIDTH = 144.0;
    public static final double FIELD_CENTER = 72.0;
    
    // Camera calibration (will be auto-detected if using webcam)
    private double[] cameraIntrinsics = null;  // fx, fy, cx, cy
    
    /**
     * Create AprilTag Detection block with default 36h11 tag family.
     * Uses default tag library from AprilTagGameDatabase.
     */
    public AprilTagDetectionBlock(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, AprilTagGameDatabase.getCenterStageTagLibrary());
    }
    
    /**
     * Create with custom tag library.
     */
    public AprilTagDetectionBlock(HardwareMap hardwareMap, Telemetry telemetry, AprilTagLibrary tagLibrary) {
        this.telemetry = telemetry;
        
        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setTagLibrary(tagLibrary)
            .setDrawAxes(false)
            .setDrawCubeProjection(false)
            .setDrawTagOutline(true)
            .setDrawTagID(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .build();
        
        currentDetections = new ArrayList<>();
        closestTag = null;
    }
    
    /**
     * Set camera intrinsics for accurate pose estimation.
     * Note: Call this BEFORE building the VisionPortal.
     * 
     * @param fx focal length x (pixels)
     * @param fy focal length y (pixels)
     * @param cx principal point x (pixels)
     * @param cy principal point y (pixels)
     */
    public void setCameraIntrinsics(double fx, double fy, double cx, double cy) {
        this.cameraIntrinsics = new double[]{fx, fy, cx, cy};
        // Note: For webcam, intrinsics are often auto-detected.
        // For explicit setting, use: aprilTagProcessor.setLensIntrinsics(fx, fy, cx, cy);
    }
    
    /**
     * Get the AprilTag processor for VisionPortal builder.
     */
    public AprilTagProcessor getProcessor() {
        return aprilTagProcessor;
    }
    
    /**
     * Update detections from latest frame.
     * Call this each loop iteration.
     */
    public void update(List<AprilTagDetection> detections) {
        currentDetections = (detections != null) ? detections : new ArrayList<>();
        closestTag = findClosestTag();
    }
    
    /**
     * Find closest tag based on detection confidence.
     */
    private AprilTagDetection findClosestTag() {
        if (currentDetections.isEmpty()) {
            return null;
        }
        
        AprilTagDetection closest = null;
        float bestMargin = Float.NEGATIVE_INFINITY;
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.decisionMargin > bestMargin) {
                bestMargin = detection.decisionMargin;
                closest = detection;
            }
        }
        return closest;
    }
    
    // ==================== Getters ====================
    
    public List<AprilTagDetection> getDetections() {
        return currentDetections;
    }
    
    public int getTagCount() {
        return currentDetections.size();
    }
    
    public AprilTagDetection getClosestTag() {
        return closestTag;
    }
    
    public AprilTagDetection getTagById(int tagId) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }
    
    public boolean isTagVisible(int tagId) {
        return getTagById(tagId) != null;
    }
    
    // ==================== Pose Information ====================
    
    /**
     * Get robot pose from a specific tag.
     * Returns null if tag not visible.
     * 
     * @return [x, y, heading] in FTC coordinates (inches, radians)
     */
    public double[] getRobotPoseFromTag(int tagId) {
        AprilTagDetection tag = getTagById(tagId);
        if (tag == null || tag.ftcPose == null) {
            return null;
        }
        
        return new double[]{
            tag.ftcPose.x,
            tag.ftcPose.y,
            tag.ftcPose.yaw
        };
    }
    
    /**
     * Get robot pose from closest tag.
     */
    public double[] getRobotPoseFromClosestTag() {
        if (closestTag == null || closestTag.ftcPose == null) {
            return null;
        }
        
        return new double[]{
            closestTag.ftcPose.x,
            closestTag.ftcPose.y,
            closestTag.ftcPose.yaw
        };
    }
    
    /**
     * Get tag field position from standard FTC layout.
     * Based on CENTERSTAGE 2024-2025 tag positions.
     * 
     * @param tagId AprilTag ID (1-21)
     * @return [x, y] in FTC coordinates, or null if unknown
     */
    public static double[] getTagFieldPosition(int tagId) {
        // These are approximate field positions - verify with game manual!
        switch (tagId) {
            // Blue Alliance Station (front)
            case 1:  return new double[]{15.5, 20.25};
            case 2:  return new double[]{15.5, 42.25};
            case 3:  return new double[]{15.5, 64.25};
            
            // Backdrop (audience side)
            case 4:  return new double[]{128.5, 86.25};
            case 5:  return new double[]{128.5, 108.25};
            case 6:  return new double[]{128.5, 130.25};
            
            // Backdrop (behind audience)
            case 7:  return new double[]{15.5, 86.25};
            case 8:  return new double[]{15.5, 108.25};
            case 9:  return new double[]{15.5, 130.25};
            
            // Center
            case 10: return new double[]{72, 42.25};
            case 11: return new double[]{72, 64.25};
            
            // Red Alliance Station (front)
            case 17: return new double[]{128.5, 20.25};
            case 18: return new double[]{128.5, 42.25};
            case 19: return new double[]{128.5, 64.25};
            
            default: return null;
        }
    }
    
    /**
     * Calculate distance from robot to a specific tag.
     * Returns -1 if tag not visible.
     */
    public double getDistanceToTag(int tagId) {
        AprilTagDetection tag = getTagById(tagId);
        if (tag == null || tag.ftcPose == null) {
            return -1;
        }
        return Math.sqrt(tag.ftcPose.x * tag.ftcPose.x + tag.ftcPose.y * tag.ftcPose.y);
    }
    
    /**
     * Get tag size for distance estimation.
     * Larger tag area = closer.
     */
    public float getTagArea(int tagId) {
        AprilTagDetection tag = getTagById(tagId);
        if (tag == null || tag.corners == null) {
            return 0;
        }
        
        // Calculate approximate area from corners
        float area = 0;
        int n = tag.corners.length;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            area += (float)(tag.corners[i].x * tag.corners[j].y);
            area -= (float)(tag.corners[j].x * tag.corners[i].y);
        }
        return Math.abs(area) / 2f;
    }
    
    // ==================== Telemetry ====================
    
    public void logToTelemetry() {
        if (currentDetections.isEmpty()) {
            telemetry.addLine("No AprilTags detected");
            return;
        }
        
        telemetry.addData("Tags", getTagCount());
        
        for (AprilTagDetection d : currentDetections) {
            String poseStr = (d.ftcPose != null) 
                ? String.format("(%.1f, %.1f) yaw=%.1f°", d.ftcPose.x, d.ftcPose.y, Math.toDegrees(d.ftcPose.yaw))
                : "no pose";
            telemetry.addData("Tag " + d.id, "M:%.1f %s", d.decisionMargin, poseStr);
        }
    }
    
    /**
     * Log detailed info for a specific tag.
     */
    public void logTagDetail(int tagId) {
        AprilTagDetection d = getTagById(tagId);
        if (d == null) {
            telemetry.addLine("Tag " + tagId + ": not visible");
            return;
        }
        
        telemetry.addLine("=== Tag " + d.id + " ===");
        telemetry.addData("Hamming", d.hamming);
        telemetry.addData("Margin", d.decisionMargin);
        
        if (d.ftcPose != null) {
            telemetry.addData("Pose", "(%.2f, %.2f, %.2f)", d.ftcPose.x, d.ftcPose.y, d.ftcPose.yaw);
        }
        
        if (d.metadata != null) {
            telemetry.addData("Size", "%.2f in", d.metadata.tagsize * 39.3701);
        }
    }
}