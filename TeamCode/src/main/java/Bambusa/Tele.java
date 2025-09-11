package Bambusa;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This is a simple sample TeleOp for Bambusa 6226.
 *
 * For those who are new,
 * The runOpMode function is where you will initialize (assign values) to your variables. This happens once.
 * The while opModeIsActive loop is what updates in real time, until the robot is stopped.
 *
 * Here are the default controls:
 *
 * Driving (With Gamepad1):
 * Translation - Left Stick - Simple forward-backward and strafing movement.
 * Rotation - Right Stick X - Simple rotation, will not affect movement direction, as this is a field-centric drive.
 * Speed Control - Left Trigger - The more this is pressed, the closer the drive speed gets to boost speed.
 * Reset IMU - Left Dpad - Sets direction of forward to current robot heading.
*/

@TeleOp
@Config
public class Tele extends LinearOpMode {
    // Telemetry For FTC Dashboard
    private FtcDashboard dashboard;

    // Declaring Robot Class
    private Robot robot;

    // Limelight 3A
    private Limelight3A limelight;

    // Robot Parameters (Editable In FTC Dashboard)
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

        // Position Constant (Sets PID Target)
        public static double pos = 0;
    }

    public static PARAMS params = new PARAMS();

    @Override
    public void runOpMode() throws InterruptedException {
        // FTC Dashboard Setup
        dashboard = FtcDashboard.getInstance();

        // Initialize Robot
        robot = new Robot(hardwareMap);

        // Setting Ticks Per Degree + Horizontal Position (Important For PID To Work)
        robot.pid.setHorizontalPos();
        robot.pid.setTicksPerDegree((double) 560 / 90);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Sample Driving Code
            robot.drive(params.driveSpeed, params.boostSpeed, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger, gamepad1.dpad_left);

            // Sample PID - Sets Values During Runtime
            robot.pid.setPID(params.p, params.i, params.d, params.f);
            robot.pid.setSAV(params.s,  params.a, params.v);
            // robot.pid.moveTo(params.pos); // Moves Robot Arm To Position

            // FTC Dashboard Telemetry
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