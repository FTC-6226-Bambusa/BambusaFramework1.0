package Bambusa.CustomActions;

import Bambusa.Robot;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/*
 * This is a sample PID movement action for Bambusa 6226, specifically for RoadRunner v1.0.
 * Learn how to use this in CustomAction.java.
*/

public class MoveArm implements Action {
    ElapsedTime timer;
    private final Robot robot;
    private final double p;

    // Constructor
    public MoveArm(Robot robot, double position) {
        this.robot = robot;
        this.p = position;
    }

    // Main Action
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        return false;
    }
}
