package Bambusa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Tele extends LinearOpMode {
    // Declaring Robot Class
    Robot robot;

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
            // Driving
            robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.dpad_left);

            // Telemetry
            telemetry.addData("Sample Telemetry Working: ", true);

            telemetry.update();
        }
    }
}