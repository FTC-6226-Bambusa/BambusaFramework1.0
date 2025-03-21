package Bambusa;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This is a simple sample TeleOp for Bambusa 6226.
 *
 * For those who are new,
 * The runOpMode function is where you will initialize (assign values) to your variables. This happens once.
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
@Config
public class Tele extends LinearOpMode {
    // Telemetry For FTC Dashboard
    TelemetryPacket packet;
    FtcDashboard dashboard;

    // Declaring Robot Class
    Robot robot;

    // PID Parameters (Editable In FTC Dashboard)
    public static class PARAMS {
        // Drive Speed & Boost Speed
        public static double driveSpeed = 0.4;
        public static double boostSpeed = 0.8;

        // PID Constants
        public static double p = 0.01;
        public static double i = 0;
        public static double d = 0;
        public static double f = 0.37;

        // Motion Profiling Constants
        public static double v = -0.00037;
        public static double a = 0.000007;
        public static double s = 0;

        // Position Constant (Sets Arm Position)
        public static double pos = 0;
    }

    public static PARAMS params = new PARAMS();

    @Override
    public void runOpMode() throws InterruptedException {
        // FTC Dashboard setup
        dashboard = FtcDashboard.getInstance();

        // Initialize Robot
        robot = new Robot(hardwareMap);

        // Setting PID Constants
        robot.pid.setHorizontalPos(5);
        // 560 is from arm down to arm vertical, 90 is what it is supposed to display
        robot.pid.setTicksPerDegree(560/90);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Sample Driving Code
            robot.drive(params.driveSpeed, params.boostSpeed, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.dpad_left);

            // Sample PID - Sets Values During Runtime
            robot.pid.setPID(params.p, params.i, params.d, params.f);
            robot.pid.setSAV(params.s,  params.a, params.v);
            robot.pid.moveTo(params.pos);

            // Create a new telemetry packet each loop
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("Current Error", robot.pid.getError());
            packet.put("Current Power", robot.pid.getPower());
            packet.put("Current Position", robot.pid.getPos());
            packet.put("Dashboard Telemetry Working", true);
            dashboard.sendTelemetryPacket(packet);

            // Driver Station Telemetry
            telemetry.addData("Current Error", robot.pid.getError());
            telemetry.addData("Current Position", robot.pid.getPos());
            telemetry.addData("Sample Telemetry Working", true);
            telemetry.update();
        }
    }

}