package Bambusa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This is a simple sample TeleOp for Bambusa 6226.
 *
 * For those who are new,
 * The runOpMode function is where you will initialize (assign values) to your variables. It is basically preparation.
 * In the while opModeIsActive loop, this is what updates every frame, until the robot is stopped.
 *
 * Here are the controls:
 *
 * Driving (With Gamepad1):
 * Translation - Left Stick - Simple forward-backward and strafing movement.
 * Rotation - Right Stick X - Simple rotation, will not affect movement direction, as this is a field-centric drive.
 * Speed Control - Left Trigger - The more this is pressed, the closer the drive speed gets to boost speed.
 * Reset IMU - Left Dpad - Sets direction of forward to current heading.
 */



@TeleOp
public class Tele extends LinearOpMode {
    // Declaring Robot Class
    Robot robot;

    // Drive Speed And Boost Speed
    public double driveSpeed = 0.4;
    public double boostSpeed = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        /// INITIALIZATION ///

        // Initializing New Robot
        robot = new Robot(hardwareMap);

        // Starting And Stopping Robot
        waitForStart();
        if (isStopRequested()) return;

        /// UPDATES ///
        while (opModeIsActive()) {
            // Sample Driving Code
            robot.drive(driveSpeed, boostSpeed, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.dpad_left);

            // Telemetry
            telemetry.addData("Sample Telemetry Working: ", true);

            telemetry.update();
        }
    }
}