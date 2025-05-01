package Bambusa.CustomActions;

import Bambusa.Robot;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/*
 * This is a sample Custom Action for Bambusa 6226, specifically for RoadRunner v1.0.
 *
 * The Auto class will have an example of how CustomAction is initialized.
 *
 * If this is your first time, understand that the action will continue to run until the 'run'
 * function returns false, in which case it will not be called again.
 *
 * Here is a sample implementation of CustomAction that would be placed in Auto.java:
 *
 *      Actions.runBlocking(
 *              drive.actionBuilder(startPose)
 *                      .stopAndAdd(new CustomAction(robot, 666))
 *                      .build()
 *      );
 *
 * Credits to 6282 Simi Valley Robotics for having an awesome tutorial on how to create custom actions.
 * Their video will be linked here: https://www.youtube.com/watch?v=uBwVSRxvpB8&t=1633s
*/

public class CustomAction implements Action {
    ElapsedTime timer;
    private final Robot robot;
    private final double p;

    // Constructor
    public CustomAction(Robot robot, double position) {
        this.robot = robot;
        this.p = position;
    }

    // Main Action
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        // Do something :)

        // Stops after 3 seconds.
        return timer.seconds() < 3;
    }
}
