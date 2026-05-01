package org.firstinspires.ftc.moon.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * MoonTeleOp - Matches original TeleopNew.java control scheme
 * Package: org.firstinspires.ftc.moon.drive
 */
@TeleOp(name = "Moon TeleOp", group = "Moon")
public class MoonTeleOp extends OpMode {

    private MoonRobot robot;
    private MultipleTelemetry joinedTelemetry;
    private TelemetryPacket telemetryPacket;

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
        joinedTelemetry.update();

        RobotLog.a("Moon TeleOp initialized");
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

        boolean relocalize = gamepad1.startWasPressed() || gamepad2.startWasPressed();
        boolean cameraOff = gamepad1.touchpadWasPressed() || gamepad2.backWasPressed();
        boolean shootRequest = gamepad2.rightBumperWasPressed();
        robot.updateFollower(relocalize, cameraOff, shootRequest, joinedTelemetry);

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

            telemetryPacket = new TelemetryPacket();
            robot.drawPose(telemetryPacket);
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        }

        joinedTelemetry.update();
    }

    @Override
    public void stop() {
        MoonRobot.clearInstance();
        RobotLog.d("Moon TeleOp stopped");
    }
}