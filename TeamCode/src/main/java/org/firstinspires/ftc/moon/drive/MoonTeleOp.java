package org.firstinspires.ftc.moon.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.moon.vision.AprilTagDetectionBlock;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * MoonTeleOp - TeleOp with AprilTag relocalization
 * Press START (gamepad) to force relocalization via AprilTag
 * Or automatically update pose when AprilTag is visible
 */
@TeleOp(name = "Moon TeleOp", group = "Moon")
public class MoonTeleOp extends OpMode {

    private MoonRobot robot;
    private MultipleTelemetry joinedTelemetry;
    private TelemetryPacket telemetryPacket;
    
    // AprilTag relocalization
    private VisionPortal visionPortal;
    private AprilTagDetectionBlock aprilTag;
    private boolean aprilTagEnabled = true;
    private int targetTagId = -1;  // -1 = use closest tag
    
    // AprilTag pose history for filtering
    private Pose[] poseHistory = new Pose[5];
    private int poseHistoryIndex = 0;
    private boolean poseHistoryFull = false;

    private boolean intake = false;
    private boolean setPose = false;
    private boolean slow = false;
    private boolean RT = false;
    private boolean LT = false;
    private boolean moveShot = false;

    @Override
    public void init() {
        joinedTelemetry = new MultipleTelemetry(
            FtcDashboard.getInstance().getTelemetry(),
            telemetry
        );

        Pose startPose = new Pose(72, 72, Math.PI / 2);
        robot = MoonRobot.createInstance(startPose, MoonRobot.Color.BLUE);
        robot.initTeleop(hardwareMap, joinedTelemetry);

        telemetryPacket = new TelemetryPacket();
        joinedTelemetry.setMsTransmissionInterval(200);
        
        // Initialize AprilTag detection
        initAprilTag();
        
        joinedTelemetry.update();
        RobotLog.a("Moon TeleOp initialized" + (aprilTagEnabled ? " with AprilTag" : ""));
    }
    
    private void initAprilTag() {
        if (!aprilTagEnabled) return;
        
        try {
            // Create AprilTag detection block
            aprilTag = new AprilTagDetectionBlock(hardwareMap, joinedTelemetry);
            
            // Build vision portal - use back camera for AprilTag viewing behind robot
            visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag.getProcessor())
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
            
            joinedTelemetry.addLine("AprilTag: Camera initialized");
        } catch (Exception e) {
            aprilTagEnabled = false;
            joinedTelemetry.addLine("AprilTag: Disabled (" + e.getMessage() + ")");
            RobotLog.e("AprilTag init failed: " + e.getMessage());
        }
    }

    @Override
    public void start() {
        robot.startTeleop();
    }

    @Override
    public void loop() {
        // LB for intake toggle (gamepad1)
        boolean lb;
        if (gamepad1.left_bumper && !intake) {
            lb = true;
        } else {
            lb = false;
        }
        intake = gamepad1.left_bumper;

        // RT for stopper open (gamepad2)
        boolean rt = gamepad2.right_trigger > 0.1 && !RT;
        RT = gamepad2.right_trigger > 0.1;

        // LT for stopper close (gamepad2)
        boolean lt = gamepad2.left_trigger > 0.1 && !LT;
        LT = gamepad2.left_trigger > 0.1;

        // X toggle setPose (both gamepads)
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            setPose = !setPose;
        }

        // LB toggle slow mode (gamepad2)
        if (gamepad2.leftBumperWasPressed()) {
            slow = !slow;
        }

        // Y toggle move shot mode (both gamepads)
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            moveShot = !moveShot;
        }

        // === STICK INPUT ===
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // === UPDATE ROBOT ===
        robot.clearBulkCache();
        
        // Check for relocalization request
        boolean relocalizeRequested = gamepad1.startWasPressed() || gamepad2.startWasPressed();
        
        // Update pose from AprilTag if available
        updatePoseFromAprilTag(relocalizeRequested);

        boolean cameraOff = gamepad1.touchpadWasPressed() || gamepad2.backWasPressed();
        boolean shootRequest = gamepad2.rightBumperWasPressed();
        robot.updateFollower(relocalizeRequested || targetTagId >= 0, cameraOff, shootRequest, joinedTelemetry);

        // Update drive (basic teleop mode - no p2p)
        robot.driveTele(
            forward,
            strafe,
            rotate,
            slow,
            false,  // p2p1
            false,  // p2p2
            false,  // p2pEnded
            joinedTelemetry
        );

        // === TELEMETRY ===
        Pose currentPose = robot.getPose();

        if (!MoonRobot.Constants.MINIMIZE_TELEMETRY) {
            joinedTelemetry.addData("--- MOON TELEOP ---", "");
            joinedTelemetry.addData("Alliance", robot.color);
            joinedTelemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            joinedTelemetry.addData("Pose X", String.format("%.2f", currentPose.getX()));
            joinedTelemetry.addData("Pose Y", String.format("%.2f", currentPose.getY()));
            joinedTelemetry.addData("Pose H", String.format("%.1f deg", Math.toDegrees(currentPose.getHeading())));
            joinedTelemetry.addData("Slow", slow);
            joinedTelemetry.addData("MoveShot", moveShot);
            joinedTelemetry.addData("SetPose", setPose);
            joinedTelemetry.addData("forward", forward);
            joinedTelemetry.addData("strafe", strafe);
            joinedTelemetry.addData("rotate", rotate);
            
            // AprilTag telemetry
            if (aprilTagEnabled) {
                AprilTagDetection closest = aprilTag.getClosestTag();
                if (closest != null) {
                    joinedTelemetry.addData("AprilTag", "ID:%d margin:%.0f", closest.id, closest.decisionMargin);
                    if (closest.ftcPose != null) {
                        joinedTelemetry.addData("AT Pose", "(%.1f, %.1f) %.1f°",
                            closest.ftcPose.x, closest.ftcPose.y, Math.toDegrees(closest.ftcPose.yaw));
                    }
                } else {
                    joinedTelemetry.addData("AprilTag", "Not visible");
                }
                joinedTelemetry.addData("Target Tag", targetTagId >= 0 ? targetTagId : "closest");
            }

            telemetryPacket = new TelemetryPacket();
            robot.drawPose(telemetryPacket);
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        }

        joinedTelemetry.update();
    }
    
    /**
     * Update robot pose from AprilTag detection.
     * Uses filtered pose for stability.
     */
    private void updatePoseFromAprilTag(boolean forceRelocalize) {
        if (!aprilTagEnabled || aprilTag == null) return;
        
        // Get current detections
        List<AprilTagDetection> detections = aprilTag.getDetections();
        aprilTag.update(detections);
        
        // Determine which tag to use
        AprilTagDetection targetTag = null;
        
        if (targetTagId >= 0) {
            // Use specific tag
            targetTag = aprilTag.getTagById(targetTagId);
        } else if (forceRelocalize || aprilTag.getTagCount() > 0) {
            // Use closest tag when forcing reloc or when tags visible
            targetTag = aprilTag.getClosestTag();
        }
        
        if (targetTag == null || targetTag.ftcPose == null) return;
        
        // Get pose from tag (FTC coordinates)
        double tagX = targetTag.ftcPose.x;
        double tagY = targetTag.ftcPose.y;
        double tagYaw = targetTag.ftcPose.yaw;
        
        // Apply offset from tag to robot center
        // Assuming camera is at front of robot, offset forward ~6 inches
        double robotX = tagX;
        double robotY = tagY;
        double robotHeading = tagYaw;
        
        // Add to pose history for filtering
        Pose newPose = new Pose(robotX, robotY, robotHeading);
        addToPoseHistory(newPose);
        
        // Get filtered pose (average of recent readings)
        Pose filteredPose = getFilteredPose();
        
        // Only update if we have valid readings
        if (filteredPose != null && (forceRelocalize || targetTagId >= 0)) {
            robot.setPose(filteredPose);
        }
    }
    
    /**
     * Add pose to rolling history for filtering.
     */
    private void addToPoseHistory(Pose pose) {
        poseHistory[poseHistoryIndex] = pose;
        poseHistoryIndex = (poseHistoryIndex + 1) % poseHistory.length;
        if (poseHistoryIndex == 0) {
            poseHistoryFull = true;
        }
    }
    
    /**
     * Get filtered pose from history (simple average).
     */
    private Pose getFilteredPose() {
        int count = poseHistoryFull ? poseHistory.length : poseHistoryIndex;
        if (count == 0) return null;
        
        double sumX = 0, sumY = 0, sumH = 0;
        int validCount = 0;
        
        for (int i = 0; i < count; i++) {
            if (poseHistory[i] != null) {
                sumX += poseHistory[i].getX();
                sumY += poseHistory[i].getY();
                sumH += Math.sin(poseHistory[i].getHeading());  // Average angular using sin/cos
                validCount++;
            }
        }
        
        if (validCount == 0) return null;
        
        // Handle angle wrapping for average
        double avgSinH = sumH / validCount;
        double avgCosH = Math.sqrt(1 - avgSinH * avgSinH);  // Approximate
        double avgH = Math.atan2(avgSinH, avgCosH);
        
        return new Pose(sumX / validCount, sumY / validCount, avgH);
    }

    @Override
    public void stop() {
        // Close vision portal to release camera
        if (visionPortal != null) {
            visionPortal.close();
        }
        
        MoonRobot.clearInstance();
        RobotLog.d("Moon TeleOp stopped");
    }
}