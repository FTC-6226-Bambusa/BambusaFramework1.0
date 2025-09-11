package Bambusa;

// Import Custom Actions Here
import Bambusa.CustomActions.MoveArm;

// Other Imports
import RoadRunner.MecanumDrive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is a sample autonomous for Bambusa 6226, which already has RoadRunner v1.0.
 *
 * If you are using this for the first time, make sure to tune RoadRunner v1.0 by going to:
 * https://rr.brott.dev/docs/v1-0/tuning/
 *
 * If your robot is not running, then it is likely that your Kv and Ka values in MecanumDrive are still 0.
 *
 * If you need to reference a different auto, I highly recommend visiting 6282 Simi Valley Robotics's youtube channel,
 * as it has multiple great tutorials on how to use RoadRunner v1.0. Their channel will be linked here:
 * https://www.youtube.com/@FTC6282
 *
 * Specific Helpful Videos (For RoadRunner v1.0):
 * Installing And Tuning: https://www.youtube.com/watch?v=3Sd1S6F35tA
 * Creating First Auto: https://www.youtube.com/watch?v=uBwVSRxvpB8&t=1633s
 * Gain Tuning Tutorial: https://www.youtube.com/watch?v=DLQDwS_EZjU
*/

@Autonomous
public class Auto extends LinearOpMode {
    // Telemetry For FTC Dashboard
    TelemetryPacket packet;
    FtcDashboard dashboard;

    // Sample Mecanum Drive
    MecanumDrive drive;
    Robot robot;

    // Perceived Starting Position (in)
    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {
        // Starting And Stopping
        waitForStart();
        if (isStopRequested()) return;

        // Autonomous Mecanum Drive
        drive = new MecanumDrive(hardwareMap, startPose);

        // Robot For Custom Actions
        robot = new Robot(hardwareMap);

        // Action Example
        Action doSomething = drive.actionBuilder(startPose)
                                  .lineToX(50) // Position In Inches
                                  .stopAndAdd(new MoveArm(robot, 260)) // Custom Action Example
                                  .waitSeconds(10)
                                  .build();

        // Sample Trajectory - Runs Automatically
        Actions.runBlocking(doSomething);

        // Telemetry (Dashboard)
        packet.put("Dashboard Telemetry Working: ", true);
        dashboard.sendTelemetryPacket(packet);

        // Telemetry (Driver Station)
        telemetry.addData("Sample Telemetry Working: ", true);
        telemetry.update();
    }
}

// According to all known laws of aviation, there is no way a bee should be able to fly. Its wings are too small to
// get its fat little body off the ground. The bee, of course, flies anyway, because bees don't care what humans think
// is impossible.

// Yellow-Black, Yellow, Black, Yellow, Black, Yellow, Black -- Ooh, Black and Yellow! Let's shake things up a little!