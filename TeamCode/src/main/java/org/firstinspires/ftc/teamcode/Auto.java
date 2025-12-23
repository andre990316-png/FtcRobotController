package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto")
public class Auto extends OpMode {

    private DcMotor IntakeMotor;
    private DcMotor ShooterM1; // right
    private DcMotor ShooterM2; // left
    private DcMotor ShooterRotateMotor;
    private Servo ShooterS1;
    private CRServo IntakeServo;

    private Limelight3A limelight;
    private IMU imu;

    private double Kp = 0.02;
    private double Kd = 0.0008;
    private double deadband = 1.0; // degrees
    private double maxTurretPower = 1;

    private double shooterPower = 0.0;
    private boolean shooterEnabled = false;

    private double lastTx = 0;
    private double lastAimTime = 0;
    private double lastTurretPower = 0;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ====== PATHS (Path1..Path11) ======
    private Paths paths;
    private int pathIndex = 2; // start from Path2 (skip Path1 because it is zero-length)

    public enum PathState {
        DriveStartToShoot,
        RunPaths,
        Done
    }
    private PathState pathState;

    private final Pose startPose = new Pose(20, 123, Math.toRadians(143));

    // ------------------------------------------------------------
    // Your Paths generator
    // ------------------------------------------------------------
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 123.000), new Pose(20.000, 123.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 123.000), new Pose(47.323, 95.865)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(137))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(47.323, 95.865), new Pose(47.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(47.000, 84.000), new Pose(15.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(15.000, 84.000), new Pose(47.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(47.000, 96.000), new Pose(47.323, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(47.323, 60.000), new Pose(12.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(12.000, 60.000), new Pose(47.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(47.000, 96.000), new Pose(47.323, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(47.323, 36.000), new Pose(12.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(12.000, 36.000), new Pose(47.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();
        }
    }

    // Build all paths once
    public void buildPaths() {
        paths = new Paths(follower);
    }

    // Get PathN by index
    private PathChain getPathByIndex(int idx) {
        if (paths == null) return null;
        switch (idx) {
            case 1: return paths.Path1;
            case 2: return paths.Path2;
            case 3: return paths.Path3;
            case 4: return paths.Path4;
            case 5: return paths.Path5;
            case 6: return paths.Path6;
            case 7: return paths.Path7;
            case 8: return paths.Path8;
            case 9: return paths.Path9;
            case 10: return paths.Path10;
            case 11: return paths.Path11;
            default: return null;
        }
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DriveStartToShoot:
                // Start the first REAL path immediately
                follower.followPath(getPathByIndex(pathIndex), true);
                pathTimer.resetTimer();
                pathIndex++;
                setPathState(PathState.RunPaths);
                break;

            case RunPaths:
                // Start the next path only when idle
                if (!follower.isBusy()) {
                    PathChain next = getPathByIndex(pathIndex);
                    if (next != null) {
                        follower.followPath(next, true);
                        pathTimer.resetTimer();
                        pathIndex++;
                    } else {
                        setPathState(PathState.Done);
                    }
                }
                break;

            case Done:
                // Optional: stop/hold here
                break;

            default:
                telemetry.addLine("No state commanded.");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DriveStartToShoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        IntakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        ShooterM1 = hardwareMap.get(DcMotor.class, "Shooter M1");
        ShooterM2 = hardwareMap.get(DcMotor.class, "Shooter M2");
        ShooterS1 = hardwareMap.get(Servo.class, "Shooter S1");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(3);
        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        buildPaths();
        follower.setPose(startPose);

        // start from Path2
        pathIndex = 2;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        limelight.start();
        lastAimTime = getRuntime();
        lastTx = 0;
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // ===== Limelight yaw update =====
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        // ===== Turret auto-aim =====
        double turretPower = 0;
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double error = -tx;

            double Aim_now = getRuntime();
            double Aim_dt = Aim_now - lastAimTime;
            if (Aim_dt <= 0) Aim_dt = 0.02; // safety

            double dTx = (tx - lastTx) / Aim_dt;

            if (Math.abs(tx) <= deadband) {
                turretPower = 0;
            } else {
                turretPower = Kp * error - Kd * dTx;
            }

            turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);

            lastTx = tx;
            lastAimTime = Aim_now;

            telemetry.addData("AutoAim", "ON");
            telemetry.addData("tx", tx);
            telemetry.addData("ta", llResult.getTa());
            telemetry.addData("dTx", dTx);
            telemetry.addData("Turret PD", turretPower);
        } else {
            turretPower = lastTurretPower;
            telemetry.addData("AutoAim", "ON (no valid tag)");
        }

        ShooterRotateMotor.setPower(turretPower);
        lastTurretPower = turretPower;

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("path index", pathIndex);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("IMU yaw", orientation.getYaw());
        telemetry.update();
    }
}
/*package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Auto")
public class Auto extends OpMode {
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private DcMotor IntakeMotor;
    private DcMotor ShooterM1; // right
    private DcMotor ShooterM2; // left
    private DcMotor ShooterRotateMotor;
    private Servo ShooterS1;
    private CRServo IntakeServo;
    private Limelight3A limelight;
    private IMU imu;
    private double Kp = 0.02;
    private double Kd = 0.0008;
    private double deadband = 1.0; // degrees
    private double maxTurretPower = 1;
    private double shooterPower = 0.0;
    private boolean shooterEnabled = false;
    private double lastTx = 0;
    private double lastAimTime = 0;
    private double lastTurretPower = 0;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        DriveStartToShoot,
        ShootPreload
    }
    PathState pathState;
    private final Pose startPose = new Pose(20, 123, Math.toRadians(143));
    private final Pose shootPose = new Pose(47.32299012693935, 95.86459802538788, Math.toRadians(137));
    private PathChain driveStartPoseShootPose;
    public void buildPaths() {
        driveStartPoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch(pathState) {
            case DriveStartToShoot:
                follower.followPath(driveStartPoseShootPose, true);
                setPathState(PathState.ShootPreload);
                break;
            case ShootPreload:
                if(follower.isBusy()) {

                }
                break;
            default:
                telemetry.addLine("No state commanded.");
                break;
        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = PathState.DriveStartToShoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        IntakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        ShooterM1 = hardwareMap.get(DcMotor.class, "Shooter M1");
        ShooterM2 = hardwareMap.get(DcMotor.class, "Shooter M2");
        ShooterS1 = hardwareMap.get(Servo.class, "Shooter S1");

        // Vision + turret
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(3);
        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        buildPaths();
        follower.setPose(startPose);
    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        limelight.start();
        lastAimTime = getRuntime();
        lastTx = 0;
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        double turretPower = 0;
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double error = -tx;

            double Aim_now = getRuntime();
            double Aim_dt = Aim_now - lastAimTime;
            //if (Aim_dt <= 0) Aim_dt = 0.02;

            //double Aim_dt = 0.04;
            double dTx = (tx - lastTx) / Aim_dt;

            if (Math.abs(tx) <= deadband) {
                turretPower = 0;
            } else {
                turretPower = Kp * error - Kd * dTx;
            }

            turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);
            shooterPower = 0.753998 - 0.146848 * Math.log(llResult.getTa());
            shooterPower = Range.clip(shooterPower, 0.0, 1.0);
            ShooterM1.setPower(shooterPower);
            ShooterM2.setPower(-shooterPower);
            lastTx = tx;
            lastAimTime = Aim_now;

            telemetry.addData("AutoAim", "ON");
            telemetry.addData("tx", tx);
            telemetry.addData("ta", llResult.getTa());
            telemetry.addData("dTx", dTx);
            telemetry.addData("Turret PD", turretPower);
        } else {
            // No valid tag -> stop turret for safety
            turretPower = lastTurretPower;
            telemetry.addData("AutoAim", "ON (no valid tag)");
        }
        ShooterRotateMotor.setPower(turretPower);
        lastTurretPower = turretPower;
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}*/
