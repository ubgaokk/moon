package org.firstinspires.ftc.moon.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.moon.drive.MoonRobot;
import org.firstinspires.ftc.moon.planner.FTCConstants;
import org.firstinspires.ftc.moon.planner.HybridAStar;
import org.firstinspires.ftc.moon.planner.PathConverter;
import org.firstinspires.ftc.moon.planner.Vec3d;

import java.util.List;

/**
 * HybridAStar Autonomous OpMode
 * Uses Hybrid A* path planner to navigate from start to goal.
 */
@Autonomous(name = "HybridA Star Auto", group = "Moon")
public class HybridAStarAuto extends OpMode {
    
    private MoonRobot robot;
    private MultipleTelemetry telemetry;
    private HybridAStar planner;
    private List<Vec3d> plannedPath;
    
    // Start/goal poses (FTC coordinates: origin at center, 0=forward/+Y)
    private static final Vec3d START = new Vec3d(10, 10, 0);
    private static final Vec3d GOAL = new Vec3d(130, 130, Math.PI / 2);
    
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
            FtcDashboard.getInstance().getTelemetry(),
            this.telemetry
        );
        telemetry.setMsTransmissionInterval(50);
        
        // Init robot at start pose (FTC coords → Pedro)
        Pose startPose = FTCConstants.ftcToPedro(START.x, START.y, START.z);
        robot = MoonRobot.createInstance(startPose, MoonRobot.Color.BLUE);
        robot.initTeleop(hardwareMap, telemetry);
        
        telemetry.addLine("[HybridA*] Initializing planner...");
        telemetry.update();
        
        initPlanner();
        
        telemetry.addLine("[HybridA*] Ready. Press PLAY to plan and follow.");
        telemetry.update();
        
        RobotLog.d("HybridAStarAuto initialized");
    }
    
    private void initPlanner() {
        // Config (tune for your robot/field)
        planner = new HybridAStar(
            15.0, 1,          // steeringAngle, steeringAngleNum
            10.0, 5,          // segmentLength, segmentNum
            12.0,             // wheelBase
            1.3, 2.5, 1.8,   // steeringPenalty, reversingPenalty, steeringChangePenalty
            12.0,             // shotDistance
            72                // headingDivisions
        );
        
        // Init with FTC field bounds (144x144 inches)
        planner.init(0, 144, 0, 144, 2.0, 1.0);
        planner.setVehicleShape(18.0, 18.0, 6.0);
        
        // Add obstacles (FTC coordinates)
        addObstacles();
    }
    
    private void addObstacles() {
        // Example: add field obstacles
        // Adjust positions for your competition field
        Vec3d obs1 = FTCConstants.ftcToInternal(50, 50, 0);
        Vec3d obs2 = FTCConstants.ftcToInternal(80, 100, 0);
        planner.setObstacle(obs1.x, obs1.y);
        planner.setObstacle(obs2.x, obs2.y);
        // planner.setObstacle(100, 60);
        // planner.setObstacle(30, 80);
    }
    
    @Override
    public void start() {
        telemetry.addLine("[HybridA*] Planning...");
        telemetry.update();
        
        // Run planner
        long startTime = System.currentTimeMillis();
        Vec3d startInternal = FTCConstants.ftcToInternal(START.x, START.y, START.z);
        Vec3d goalInternal = FTCConstants.ftcToInternal(GOAL.x, GOAL.y, GOAL.z);
        plannedPath = planner.search(startInternal, goalInternal);
        long planTime = System.currentTimeMillis() - startTime;
        
        if (plannedPath.isEmpty()) {
            telemetry.addLine("[HybridA*] ERROR: No path found!");
            telemetry.update();
            RobotLog.d("HybridAStarAuto: No path found");
            return;
        }
        
        // Convert to PathChain
        PathChain pathChain = PathConverter.toPathChain(plannedPath, null);
        
        if (pathChain == null) {
            telemetry.addLine("[HybridA*] ERROR: Path conversion failed!");
            telemetry.update();
            return;
        }
        
        // Log results
        telemetry.addLine("[HybridA*] Path found!");
        telemetry.addData("  Plan time", planTime + "ms");
        telemetry.addData("  Path points", plannedPath.size());
        telemetry.addData("  Path length", String.format("%.1f in", planner.getPathLength()));
        telemetry.update();
        
        RobotLog.d("HybridAStarAuto: " + plannedPath.size() + " points in " + planTime + "ms");
        
        // Start following
        robot.followPath(pathChain, true);
    }
    
    @Override
    public void loop() {
        robot.updatePose();
        
        Pose pose = robot.getPose();
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°",
            pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        
        if (robot.atParametricEnd()) {
            telemetry.addLine("[HybridA*] Done!");
        }
    }
}