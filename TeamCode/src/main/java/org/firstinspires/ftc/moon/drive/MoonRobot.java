package org.firstinspires.ftc.moon.drive;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.Drivetrain;
import com.pedropathing.ErrorCalculator;
import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * MoonRobot - Drive system extracted from DECODE V2
 * Package: org.firstinspires.ftc.moon.drive
 */
@Config
public class MoonRobot {
    // ==================== CONSTANTS (exact copy from DECODE V2) ====================
    public static class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
                .forwardZeroPowerAcceleration(-35.356022741184866)
                .lateralZeroPowerAcceleration(-69)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0.0, 0.00001, 0.6, 0.0))
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0.0, 0.000005, 0.6, 0.0))
                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.001, 0))
                .centripetalScaling(0.0005)
                .mass(12.066);

        public static com.pedropathing.ftc.drivetrains.MecanumConstants driveConstants = new com.pedropathing.ftc.drivetrains.MecanumConstants()
                .maxPower(1)
                .xVelocity(68)
                .yVelocity(55.8)
                .rightFrontMotorName("W_FR")
                .rightRearMotorName("W_BR")
                .leftRearMotorName("W_BL")
                .leftFrontMotorName("W_FL")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

        public static com.pedropathing.ftc.localization.constants.PinpointConstants localizerConstants = new com.pedropathing.ftc.localization.constants.PinpointConstants()
                .forwardPodY(0.945)
                .strafePodX(1.488)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1.5);

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new com.pedropathing.ftc.FollowerBuilder(followerConstants, hardwareMap)
                    .pinpointLocalizer(localizerConstants)
                    .pathConstraints(pathConstraints)
                    .mecanumDrivetrain(driveConstants)
                    .build();
        }

        public static double drivePower_Tele = 1.0;
        public static double drivePower_Slow = 0.3;
        public static double driveRotationPower = 0.8;
        public static final double FIELD_OFFSET = 72;

        public static boolean MINIMIZE_TELEMETRY = false;
    }

    // ==================== SINGLETON ====================
    private static MoonRobot instance = null;

    public enum Color { RED, BLUE }
    public enum OpModeState { AUTO, TELEOP }

    public Color color = Color.BLUE;
    public OpModeState opModeState = OpModeState.TELEOP;
    public AprilFollower follower;
    private List<LynxModule> allHubs;

    private int driveSideSign = -1;
    private double drivePower = Constants.drivePower_Tele;

    private Pose txWorldPinpoint = new Pose(0, 0, Math.PI);
    private com.qualcomm.robotcore.util.ElapsedTime aprilTimer;

    // ==================== CONSTRUCTOR ====================
    public MoonRobot(Pose initialPose, Color color) {
        this.color = color;
        if (this.color == Color.RED) {
            driveSideSign = -1;
        } else if (this.color == Color.BLUE) {
            driveSideSign = 1;
        }
        this.txWorldPinpoint = initialPose;
    }

    public static MoonRobot getInstance() {
        return instance;
    }

    public static MoonRobot createInstance(Pose initialPose, Color color) {
        if (instance == null || instance.opModeState == OpModeState.TELEOP) {
            instance = new MoonRobot(initialPose, color);
        }
        return instance;
    }

    public static void clearInstance() {
        instance = null;
    }

    // ==================== INITIALIZATION ====================
    public void initTeleop(HardwareMap hardwareMap, Telemetry telemetry) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = new AprilFollower(Constants.createFollower(hardwareMap));
        follower.setPose(FTCToPedro(txWorldPinpoint));

        aprilTimer = new com.qualcomm.robotcore.util.ElapsedTime();
        drivePower = Constants.drivePower_Tele;

        RobotLog.d("MoonRobot teleop initialized for " + color);
    }

    public void startTeleop() {
        follower.startTeleopDrive(true);
    }

    // ==================== DRIVE CONTROL ====================
    public void driveTele(double forward, double right, double rotate, boolean slowMode,
                          boolean p2p1, boolean p2p2, boolean p2pEnded,
                          Telemetry telemetry) {
        if (p2pEnded) {
            follower.startTeleopDrive(true);
        }

        if (!(p2p1 || p2p2)) {
            if (slowMode) {
                drivePower = Constants.drivePower_Slow;
            } else {
                drivePower = Constants.drivePower_Tele;
            }

            follower.setTeleOpDrive(
                    forward * driveSideSign * drivePower,
                    right * driveSideSign * drivePower,
                    -rotate * Constants.driveRotationPower * drivePower,
                    true
            );
            telemetry.addData("inner forward", forward * driveSideSign * drivePower);
            telemetry.addData("inner right", right * driveSideSign * drivePower);
        } else if (p2p1) {
            follower.holdPoint(FTCToPedro(new Pose(58, 58, Math.toRadians(30))));
        } else if (p2p2) {
            follower.holdPoint(new Pose(58, 58, Math.toRadians(30)));
        }
    }

    public void drive(double forward, double strafe, double rotate, boolean slow) {
        if (slow) {
            drivePower = Constants.drivePower_Slow;
        } else {
            drivePower = Constants.drivePower_Tele;
        }

        follower.setTeleOpDrive(
                forward * driveSideSign * drivePower,
                strafe * driveSideSign * drivePower,
                -rotate * Constants.driveRotationPower * drivePower,
                false
        );
    }

    // ==================== PATH FOLLOWING ====================
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    // ==================== POSE MANAGEMENT ====================
    public void updatePose() {
        follower.updatePose();
    }

    public Pose getPose() {
        return PedroToFTC(follower.getPose());
    }

    public Pose getRawPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        txWorldPinpoint = pose;
        follower.setPose(FTCToPedro(pose));
    }

    // ==================== UTILITY ====================
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return follower.atPose(pose, xTolerance, yTolerance, headingTolerance);
    }

    public boolean atParametricEnd() {
        return follower.atParametricEnd();
    }

    public Path getCurrentPath() {
        return follower.getCurrentPath();
    }

    public PathChain getCurrentPathChain() {
        return follower.getCurrentPathChain();
    }

    public void holdPoint(BezierPoint point, double heading) {
        follower.holdPoint(point, heading);
    }

//    public void holdPoint(Pose point, double heading) {
//        follower.holdPoint(point, heading);
//    }

    public void updateFollower(boolean relocalize, boolean cameraOff, boolean shootRequest,
                               Telemetry telemetry) {
        follower.update();
        if (relocalize) {
            RobotLog.d("Relocalize triggered");
        }
    }

    public void drawPose(TelemetryPacket packet) {
        // Pose visualization placeholder
    }

    // ==================== COORDINATE CONVERSION ====================
    public Pose FTCToPedro(Pose pose) {
        Pose rotated = pose.rotate(-Math.PI / 2, true);
        return rotated.plus(new Pose(Constants.FIELD_OFFSET, Constants.FIELD_OFFSET));
    }

    public Pose PedroToFTC(Pose pedroPose) {
        Pose normalized = pedroPose.minus(new Pose(Constants.FIELD_OFFSET, Constants.FIELD_OFFSET));
        return normalized.rotate(Math.PI / 2, true);
    }

    public Pose mirrorPose(Pose pose) {
        return new Pose(pose.getX(), -pose.getY() + 2 * Constants.FIELD_OFFSET, pose.getHeading());
    }

    // ==================== APRILFOLLOWER WRAPPER ====================
    public class AprilFollower {
        private final Follower base;

        public AprilFollower(Follower base) {
            this.base = base;
        }

        public void setPose(Pose pose) {
            base.setPose(pose);
        }

        public Pose getPose() {
            return base.getPose();
        }

        public void startTeleopDrive() {
            base.startTeleopDrive();
        }

        public void startTeleopDrive(boolean useBrakeMode) {
            base.startTeleopDrive(useBrakeMode);
        }

        public void update() {
            base.update();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn) {
            base.setTeleOpDrive(forward, strafe, turn);
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric);
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric, offsetHeading);
        }

        public void followPath(Path path, boolean holdEnd) {
            base.followPath(path, holdEnd);
        }

        public void followPath(PathChain pathChain, boolean holdEnd) {
            base.followPath(pathChain, holdEnd);
        }

        public void followPath(PathChain pathChain) {
            base.followPath(pathChain);
        }

        public void updatePose() {
            base.updatePose();
        }

        public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
            return base.atPose(pose, xTolerance, yTolerance);
        }

        public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
            return base.atPose(pose, xTolerance, yTolerance, headingTolerance);
        }

        public boolean atParametricEnd() {
            return base.atParametricEnd();
        }

        public Path getCurrentPath() {
            return base.getCurrentPath();
        }

        public PathChain getCurrentPathChain() {
            return base.getCurrentPathChain();
        }

        public Vector getDriveVector() {
            return base.getDriveVector();
        }

        public Vector getTeleopDriveVector() {
            return base.getTeleopDriveVector();
        }

        public String[] debug() {
            return base.debug();
        }

        public void holdPoint(BezierPoint point, double heading) {
            base.holdPoint(point, heading);
        }
        public void holdPoint(Pose pose) {
            base.holdPoint(pose);
        }

//        public void holdPoint(Pose point, double heading) {
//            base.holdPoint(point, heading);
//        }

        public void updateDrivetrain() {
            base.updateDrivetrain();
        }

        public boolean isRobotStuck() {
            return base.isRobotStuck();
        }

        public Pose getPointFromPath(double t) {
            return base.getPointFromPath(t);
        }

        public Vector getClosestPointTangentVector() {
            return base.getClosestPointTangentVector();
        }

        public int getChainIndex() {
            return base.getChainIndex();
        }

        public void setMaxPower(double set) {
            base.setMaxPower(set);
        }

        public void updateConstants() {
            base.updateConstants();
        }
    }
}