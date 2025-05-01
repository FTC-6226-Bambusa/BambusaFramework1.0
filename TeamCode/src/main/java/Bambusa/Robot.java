package Bambusa;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/*
 * This is a sample Robot which has a built in Mecanum Drive for Bambusa 6226
 *
 * This class should contain all the motors in the entire robot, and functions that operate them accordingly.
 * Already has a field-centric drive function.
 *
 * If this is your first time, make sure to change the motor names in the Robot constructor to
 * match their names in the configuration (found in the driver station).
*/



public class Robot {
    // IMU (For Field Centric Drive)
    public IMU imu;

    // Drive Chain Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    // Arm & PID
    public DcMotorEx arm;
    public PID pid;

    /// ROBOT CONSTRUCTOR ///
    public Robot(HardwareMap hardwareMap) {
        // IMU (For Field Centric Drive)
        imu = hardwareMap.get(IMU.class, "imu");

        // Drive Train Motors
        this.frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        this.backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        this.frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        this.backRightMotor = hardwareMap.dcMotor.get("rightRear");

        // PID Use Example
        this.arm = hardwareMap.get(DcMotorEx.class, "slideChain");
        this.pid = new PID(this.arm, 0.015, 0, 0, 0, 0, 0, 0);

        // Setting Drive Train Motor Direction
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Other Motors (Example)
        // this.randomMotor = hardwareMap.get(DcMotor.class, "randomMotor");

        // Other Encoders (Example)
        // this.randomMotor = hardwareMap.get(DcMotorEx.class, "randomMotor");
        // this.randomMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        double x = leftx * 1.1;
        double rx = rightx;

        // Resets IMU Rotation
        if (resetYaw) {
            imu.resetYaw();
        }

        // Rotation Of Robot
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotates Movement Direction To Counter Bot Rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Counters Imperfect Strafing
        rotX *= 1.1;

        // Drive Train Calculations
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Final Speed
        double finalSpeed = lerp(driveSpeed, boostSpeed, lt);

        // Apply Power
        frontLeftMotor.setPower(frontLeftPower * (finalSpeed + lt * finalSpeed));
        backLeftMotor.setPower(backLeftPower * (finalSpeed + lt * finalSpeed));
        frontRightMotor.setPower(frontRightPower * (finalSpeed + lt * finalSpeed));
        backRightMotor.setPower(backRightPower * (finalSpeed + lt * finalSpeed));
    }

    // Sets Motor Powers
    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    // Sets Motor Powers To Zero
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    // Finds T Of The Way From Value A To Value B
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
