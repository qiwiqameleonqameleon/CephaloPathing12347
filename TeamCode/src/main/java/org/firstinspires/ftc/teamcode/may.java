package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous
public class may extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final Pose startPose = new Pose(72, 120, Math.toRadians(90)); //adjust x, y, and theta accordingly
    private final Pose scorePose = new Pose(72, 20, Math.toRadians(115));
    private final Pose PPGPose = new Pose(100, 83.5, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private PathChain getPPG; //ID 23
    private PathChain lebronPPG;
    private PathChain getPGP; //ID 22
    private PathChain lebronPGP;
    private PathChain getGPP; //ID 21
    private PathChain lebronGPP;
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    //yuhhh
    private Pose currentPose;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathStatePPG;
    private int pathStatePGP;
    private int pathStateGPP;
    private int foundID;

    private void log(String caption, Object...text){
        if (text.length == 1){
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption,": " + text[0]); // +?
        }
        else if (text.length >= 1) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++){
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption, ": " + text[0]); // +?
        }
    }

    public void intakeArtifacts() {
        //intake logic/functions AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHH
    }

    public void shootArtifacts() {
        //shooting logic/functions AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHH
    }



    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        boolean targetFound = false;
        init();

        if (USE_WEBCAM) {
            setManualExposure(6,250);
        }

        log("Status", "Initialized");
        telemetry.update();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1")) //USE Webcam 1
                .setCameraResolution(new Size(640,480))
                .build();

        waitForStart();
        runtime.reset();

        setPathStateGPP(0);
        setPathStatePGP(0);
        setpathStatePPG(0);
        runtime.reset();

        while(opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();
            targetFound = false;
            desiredTag = null;


            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == PPG_TAG_ID) {
                        buildPathsPPG();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 21;
                        break;
                    }
                } else if (detection.metadata != null) {
                    if (detection.id == PGP_TAG_ID) {
                        buildPathsPGP();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 22;
                        break;
                    }
                } else if (detection.metadata != null) {
                    if (detection.id == GPP_TAG_ID) {
                        buildPathsGPP();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 23;
                        break;
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }

                if (foundID == 21) {
                    updateStateMachinePPG();
                } else if (foundID == 22) {
                    updateStateMachinePGP();
                } else if (foundID == 23) {
                    updateStateMachineGPP();
                }

                log("Elapsed", runtime.toString());
                log("X", currentPose.getX());
                log("Y", currentPose.getY());
                log("Heading", currentPose.getHeading());
                telemetry.update();
            }

        }
        /*
        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                //int tagId = tagProcessor.getDetections(1).tag.id;
               // telemetry.addData("Detected Tag ID", tagId);
                telemetry.addData("x",tag.ftcPose.x);
                telemetry.addData("y",tag.ftcPose.y);
                telemetry.addData("z",tag.ftcPose.z);
                telemetry.addData("roll",tag.ftcPose.roll);
                telemetry.addData("pitch",tag.ftcPose.pitch);
                telemetry.addData("yaw",tag.ftcPose.yaw);
                telemetry.addData("ID",foundID);
                telemetry.addData("ID2",desiredTag);
            }
            telemetry.update();

            // if
        } */
    }

    public void buildPathsPPG() {
        getPPG = follower.pathBuilder()
            .addPath(new BezierLine(startPose, PPGPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), PPGPose.getHeading())
            .build();

        lebronPPG = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, scorePose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsPGP() {
        getPGP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PGPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PGPPose.getHeading())
                .build();

        lebronPGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, scorePose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsGPP() {
        getGPP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, GPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
                .build();

        lebronGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, scorePose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void updateStateMachinePPG() {
        switch(pathStatePPG) {
            case 0:
                follower.followPath(getPPG);
                setpathStatePPG(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(lebronPPG);
                    setpathStatePPG(-1);
                }
                break;
        }
    }

    public void updateStateMachinePGP() {
        switch(pathStatePGP) {
            case 0:
                follower.followPath(getPGP);
                setPathStatePGP(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(lebronPGP);
                    setPathStatePGP(-1);
                }
                break;
        }
    }

    public void updateStateMachineGPP() {
        switch(pathStateGPP) {
            case 0:
                follower.followPath(getGPP);
                setPathStateGPP(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(lebronGPP);
                    setPathStateGPP(-1);
                }
                break;
        }
    }

    void setpathStatePPG(int newPathState) {
        this.pathStatePPG = newPathState;
    }

    void setPathStatePGP(int newPathState) {
        this.pathStatePGP = newPathState;
    }

    void setPathStateGPP(int newPathState) {
        this.pathStateGPP = newPathState;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        }
        else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) {
            return;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera","Ready");
            telemetry.update();
        }
    }
}
