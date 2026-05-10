package org.firstinspires.ftc.moon.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.moon.drive.MoonRobot;
import org.firstinspires.ftc.moon.vision.AprilTagDetectionBlock;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * Example OpMode using AprilTag Detection
 * 
 * Demonstrates how to:
 * 1. Initialize VisionPortal with AprilTag processor
 * 2. Update and process detections each loop
 * 3. Use tag poses for localization
 * 
 * Prerequisites:
 * - Camera configured for AprilTag detection in driver station
 * - 36h11 tag family selected in LimeLight/camera settings
 */
@Autonomous(name = "AprilTag Example", group = "Moon")
public class AprilTagExample extends OpMode {
    
    private MultipleTelemetry telemetry;
    private VisionPortal visionPortal;
    private AprilTagDetectionBlock aprilTag;
    private MoonRobot robot;
    
    // Target tag ID to look for
    private static final int TARGET_TAG_ID = 1;
    
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
            FtcDashboard.getInstance().getTelemetry(),
            this.telemetry
        );
        
        // Initialize robot at origin (will be updated via AprilTag)
        robot = MoonRobot.createInstance(
            new com.pedropathing.geometry.Pose(0, 0, 0),
            MoonRobot.Color.BLUE
        );
        // Note: initAuto or initTeleop should be called here based on your setup
        
        // Create AprilTag detection block
        aprilTag = new AprilTagDetectionBlock(hardwareMap, telemetry);
        
        // Build vision portal with AprilTag processor
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag.getProcessor())
            .setStreamFormat(VisionPortal.StreamFormat.YUY2)
            .build();
        
        telemetry.addLine("AprilTag Example initialized");
        telemetry.addLine("Point camera at AprilTags");
        telemetry.update();
    }
    
    @Override
    public void init_loop() {
        // Wait for camera to start
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Camera ready");
        } else {
            telemetry.addData("Camera state", visionPortal.getCameraState());
        }
        telemetry.update();
    }
    
    @Override
    public void start() {
        telemetry.addLine("Started - looking for tags");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Get current detections
        List<AprilTagDetection> detections = aprilTag.getDetections();
        aprilTag.update(detections);
        
        // Log to telemetry
        aprilTag.logToTelemetry();
        
        // Check if target tag is visible
        if (aprilTag.isTagVisible(TARGET_TAG_ID)) {
            telemetry.addLine("TARGET TAG VISIBLE!");
            
            // Get robot pose from tag
            double[] robotPose = aprilTag.getRobotPoseFromTag(TARGET_TAG_ID);
            if (robotPose != null) {
                telemetry.addData("Robot pose", "(%.1f, %.1f) %.1f°",
                    robotPose[0], robotPose[1], Math.toDegrees(robotPose[2]));
                
                // Update robot position
                // robot.setPose(new Pose(robotPose[0], robotPose[1], robotPose[2]));
            }
        } else {
            telemetry.addLine("Target tag not visible");
        }
        
        // Show closest tag info
        AprilTagDetection closest = aprilTag.getClosestTag();
        if (closest != null) {
            telemetry.addData("Closest tag", closest.id);
        }
        
        telemetry.update();
    }
    
    @Override
    public void stop() {
        // Close vision portal to release camera
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}