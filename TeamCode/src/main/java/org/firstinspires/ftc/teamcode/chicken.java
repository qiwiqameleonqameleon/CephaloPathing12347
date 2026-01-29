/*package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class chicken {
    @TeleOp(name = "teleop", group = "Linear Opmode")
    public class MecanumTeleOpBumpers extends LinearOpMode {

        // Declare motor variables
        private MotorEx front_left, front_right, back_left, back_right;

        @Override
        public void runOpMode() {

            // Initialize motors from hardware map configuration
            front_left = new MotorEx(hardwareMap, "front_left");
            front_right = new MotorEx(hardwareMap, "front_right");
            back_left = new MotorEx(hardwareMap, "back_left");
            back_right = new MotorEx(hardwareMap, "back_right");

            // Reverse the direction of the right side motors
            front_right.setInverted(true);
            back_right.setInverted(true);

            // Send a "Ready" message to the Driver Station
            telemetry.addLine("TeleOp Ready");
            telemetry.update();

            // Wait for the driver to press the START button
            waitForStart();

            // This loop runs repeatedly after START is pressed
            while (opModeIsActive()) {

                // Get controller inputs
                double y = -gamepad1.left_stick_y; // Reads forward/backward movement
                double rx = gamepad1.right_stick_x; // Reads rotation

                // Check bumpers for strafing
                double x; // This variable holds the strafe power
                if (gamepad1.left_bumper) {
                    x = -1.0; // Strafe Left
                } else if (gamepad1.right_bumper) {
                    x = 1.0; // Strafe Right
                } else {
                    x = 0.0; // Don't strafe
                }

                // Calculate the power for each wheel
                double front_leftPower = y + x + rx;
                double back_leftPower = y - x + rx;
                double front_rightPower = y - x - rx;
                double back_rightPower = y + x - rx;

                // Find the highest motor power (absolute value)
                double max = Math.max(Math.abs(front_leftPower), Math.max(Math.abs(back_leftPower),
                        Math.max(Math.abs(front_rightPower), Math.abs(back_rightPower))));


                // Normalize all powers if any value is greater than 1.0
                if (max > 1.0) {
                    front_leftPower /= max;
                    back_leftPower /= max;
                    front_rightPower /= max;
                    back_rightPower /= max;
                }

                // Send the final power values to the motors
                front_left.set(front_leftPower);
                front_right.set(front_rightPower);
                back_left.set(back_leftPower);
                back_right.set(back_rightPower);


//testing something 3
                // Display values on the Driver Station for debugging
                telemetry.addData("X (Strafe)", x);
                telemetry.addData("Y (Forward)", y);
                telemetry.addData("RX (Rotation)", rx);
                telemetry.addData("---", "---"); // Visual separator
                telemetry.addData("Front Left Power", front_leftPower);
                telemetry.addData("Front Right Power", front_rightPower);
                telemetry.addData("Back Left Power", back_leftPower);
                telemetry.addData("Back Right Power", back_rightPower);
                telemetry.update(); // Pushes all telemetry data to the screen
            }
        }
    }

}
*/