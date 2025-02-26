package Bambusa;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// TODO: Make Custom Actions

/*
 * This is a sample Autonomous for Bambusa 6226, which already has RoadRunner v1.0.
 *
 * If you are using this for the first time, make sure to tune RoadRunner v1.0 by going to:
 * https://rr.brott.dev/docs/v1-0/tuning/
 *
 * If your robot is not running, then this is likely that your Kv and Ka values are still 0.
 */



@Autonomous
public class Auto extends LinearOpMode {
    // Sample Mecanum Drive
    MecanumDrive drive;

    // Perceived Starting Position (Set Later)
    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {

        // Starting And Stopping Robot
        waitForStart();
        if (isStopRequested()) return;

        // Autonomous Mecanum Drive
        drive = new MecanumDrive(hardwareMap, startPose);

        // Sample Trajectory - Runs Automatically
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToX(100)
                        .waitSeconds(20)
                        .build()
        );

        // Telemetry
        telemetry.addData("Sample Telemetry Working: ", true);

        telemetry.update();
    }
}