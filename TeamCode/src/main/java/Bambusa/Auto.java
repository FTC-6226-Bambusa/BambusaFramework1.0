package Bambusa;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Auto extends LinearOpMode {
    // Sample Mecanum
    MecanumDrive drive;

    // Perceived Starting Position (Set Later)
    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        // Starting And Stopping Robot
        waitForStart();
        if (isStopRequested()) return;

        // Autonomous Mecanum Drive
        drive = new MecanumDrive(hardwareMap, startPose);

        // Example Motor
        DcMotor randomMotor = hardwareMap.dcMotor.get("random motor");

        // Sample Trajectory
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToX(10)
                        .waitSeconds(3)
                        .stopAndAdd(new customAction(randomMotor, 0.5))
                        .build()
        );
    }

    // Sample Custom Action
    public class customAction implements Action {

        // Action Variables
        ElapsedTime timer;
        DcMotor motor;
        double power;

        // Action Constructor
        public customAction(DcMotor m, double p) {
            this.motor = m;
            this.power = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Example Code:
            if (timer == null) {
                timer = new ElapsedTime();
            }

            // Action
            motor.setPower(power);

            // Stops Action After 5 Seconds
            return timer.seconds() < 5;
        }
    }
}