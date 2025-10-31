package Bambusa;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

/*
 * This is a sample Robot which has a built in Mecanum Drive for Bambusa 6226
 *
 * This class should contain all the motors in the entire robot, and functions that operate them accordingly.
 * Already has a built-in drive functions.
 *
 * If this is your first time, make sure to change the motor names in the Robot constructor to
 * match their names in the configuration (found in the driver station).
*/

public class Robot {
    // Limelight (Camera)
    public Limelight3A limelight;

    // Inertial Measurement Unit (For Field Centric Drive)
    public IMU imu;

    // Location Of Robot
    public Pose2d pose;

    // Drive Chain Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    // Launcher Wheels
    public DcMotor launcherLeft;
    public DcMotor launcherRight;

    /// ROBOT CONSTRUCTOR ///
    public Robot(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "lemonlight");

        // IMU (For Field Centric Drive)
        imu = hardwareMap.get(IMU.class, "imu");

        // Drive Train Motors
        this.frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        this.backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        this.frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        this.backRightMotor = hardwareMap.dcMotor.get("rightRear");

        // Launcher
        // this.launcherLeft = hardwareMap.dcMotor.get("launcherLeft");
        // this.launcherRight = hardwareMap.dcMotor.get("launcherRight");

        // Setting Drive Train Motor Direction
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setting Launcher Direction
//        this.launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Adjust Orientation Parameters For IMU And Field Centric Drive
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);
    }

    // Field Centric Drive
    public void drive(double driveSpeed, double boostSpeed, double leftx, double lefty, double rightx, double lt, boolean resetYaw) {
        // User Input (Gamepad Values Are Passed As Parameters)
        double y = -lefty;
        double x = -leftx * 1.1;
        double rx = rightx;

        // Resets IMU Rotation
        if (resetYaw) {
            imu.resetYaw();
        }

        // Rotation Of Robot
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotates Movement Direction To Counter Bot Rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Counters Imperfect Strafing
        rotX *= 1.1;

        // Calculations
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Final Speed
        double finalSpeed = MathPlus.lerp(driveSpeed, boostSpeed, lt);

        // Apply Power
        frontLeftMotor.setPower(frontLeftPower * (finalSpeed + lt * finalSpeed));
        backLeftMotor.setPower(backLeftPower * (finalSpeed + lt * finalSpeed));
        frontRightMotor.setPower(frontRightPower * (finalSpeed + lt * finalSpeed));
        backRightMotor.setPower(backRightPower * (finalSpeed + lt * finalSpeed));
    }

    // Simple Strafe Drive
    public void drive(double power, double angle) {
        // Rotation
        double rotY = power * Math.cos(angle);
        double rotX = power * Math.sin(angle);

        // Calculations
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX), 1.0);
        double frontLeftPower = (rotY + rotX) / denominator;
        double backLeftPower = (rotY - rotX) / denominator;
        double frontRightPower = (rotY - rotX) / denominator;
        double backRightPower = (rotY + rotX) / denominator;

        // Applying Power
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    // Launching Ball
    public void launch() {
//        launcherLeft.setPower(1);
//        launcherLeft.setPower(0);
    }

    // Sets Motor Powers
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    // Sets Motor Powers To Zero
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    // April Tag - Detecting Ball Pattern
    public int detectFirstAprilTag(int pipeline) {
        int order1 = 10;
        int order2 = 130;
        int order3 = 50;

        limelight.pipelineSwitch(pipeline);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTagResults = result.getFiducialResults();

            if (!aprilTagResults.isEmpty()) {
                LLResultTypes.FiducialResult firstTag = aprilTagResults.get(0);

                int id = firstTag.getFiducialId();

                return id;
            }
        }

        return -1;
    }
}
