package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.7); // change later
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf") // change later
            .rightRearMotorName("rr") // change later
            .leftRearMotorName("lr") // change later
            .leftFrontMotorName("lf") // change later
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.002344161544640462)
            .strafeTicksToInches(.0029095147107972346)
            .turnTicksToInches(.001972661219346346)
            .leftPodY(7.00787402)  // change later
            .rightPodY(-7.00787402) // change later
            .strafePodX(-7.28346457) // change later
            .leftEncoder_HardwareMapName("left") // change later
            .rightEncoder_HardwareMapName("right") // change later
            .strafeEncoder_HardwareMapName("strafe") // change later
            .leftEncoderDirection(Encoder.FORWARD) // change later
            .rightEncoderDirection(Encoder.FORWARD) // change later
            .strafeEncoderDirection(Encoder.REVERSE); // change later
            //.IMU_HardwareMapName("imu")
            //.IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));  // change later
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
