/*
package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class spring extends OpMode { //huh
    private Limelight3A camera;
    private Follower follower;
    private Boolean following = false;
    private final Pose TARGET_LOCATION = new Pose();

    @Override
    public void init() {
        camera = hardwareMap.get(Limelight3A.class, "Webcam 1"); // check config
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
    }
    @Override
    public void start() {
        camera.start();
    }
    @Override
    public void loop() {
        follower.update();
        if (!following) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                            .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                            .build()
            );
        }
        follower.setPose(getRobotPoseFromCamera());
        if (following && !follower.isBusy()) following = false;
    }

    private Pose getRobotPoseFromCamera() {
        return new Pose (0,0,0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
            */