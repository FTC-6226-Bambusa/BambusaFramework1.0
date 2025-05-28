package Bambusa.Tests;

import Bambusa.PID;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
public class PIDTest extends LinearOpMode {
    // Telemetry For FTC Dashboard
    FtcDashboard dashboard;

    // Declaring Robot Class
    PID pid;

    // PID Parameters (Editable In FTC Dashboard)
    public static class PARAMS {
        // PID Constants
        public static double p = 0.01;
        public static double i = 0;
        public static double d = 0;
        public static double f = 0.37;

        // Motion Profiling Constants
        public static double v = -0.00037;
        public static double a = 0.000007;
        public static double s = 0;

        // Target Position (Angle)
        public static double pos = 0;

        // Ticks Per Degree
        public static double tpd = (double) 560 / 90;
    }

    public static PARAMS params = new PARAMS();

    @Override
    public void runOpMode() throws InterruptedException {
        // FTC Dashboard Setup
        dashboard = FtcDashboard.getInstance();

        // Initialize PID
        pid = new PID(hardwareMap.get(DcMotorEx.class, "armMotor"));
        pid.setTicksPerDegree(params.tpd);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Sample PID - Sets Values During Runtime
            pid.setPID(params.p, params.i, params.d, params.f);
            pid.setSAV(params.s,  params.a, params.v);
            pid.moveTo(params.pos); // Moves To Position

            // FTC Dashboard Telemetry
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("Current Error", pid.getError());
            packet.put("Current Power", pid.getPower());
            packet.put("Current Position", pid.getPos());
            packet.put("Current Target", params.pos);
            dashboard.sendTelemetryPacket(packet);

            // Driver Station Telemetry
            telemetry.addData("Current Error", pid.getError());
            telemetry.addData("Current Position", pid.getPos());
            telemetry.update();
        }
    }
}