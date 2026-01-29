/*package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class autotag extends OpMode {
    AprilTag aprilTagWebcam = new WebcamName();
    @Override
    public void init() {
        WebcamName.init(hardwareMap,telemetry);
    }
    @Override
    public void loop() {
        WebcamName.update();
        AprilTagDetection id20 = WebcamName.getTagBySpecificId(20);
        telemetry.addData("id20 String", id20.toString());
    }


}
*/