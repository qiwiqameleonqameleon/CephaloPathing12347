package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

   /* public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("front_left")
            .rightFrontMotorName("front_right")
            .leftRearMotorName("back_left")
            .rightRearMotorName("back_right")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);*/
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
           .forwardEncoder_HardwareMapName("frontLeft")
           .strafeEncoder_HardwareMapName("backRight")
           .forwardEncoderDirection(Encoder.REVERSE)
           .strafeEncoderDirection(Encoder.REVERSE)
           .forwardPodY(3)
           .strafePodX(6)
           .forwardTicksToInches(0.0006)
           .strafeTicksToInches(0.0007)
           .IMU_HardwareMapName("imu")
           .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
            );
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5) // weigh the robot properly
            .forwardZeroPowerAcceleration(30) // deceleration
            .lateralZeroPowerAcceleration(30); //deceleration

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(48)
            .yVelocity(48)
            .leftFrontMotorName("frontLeft")
            .rightFrontMotorName("frontRight")
            .leftRearMotorName("backLeft")
            .rightRearMotorName("backRight")


            //double check on reverses and allat

            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                //.driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}