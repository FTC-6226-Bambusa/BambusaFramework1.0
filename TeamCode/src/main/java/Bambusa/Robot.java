package Bambusa;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {

    /// VARIABLES ///
    // IMU (For Field Centric Drive)
    public IMU imu;

    // Drive Chain Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    // Other Motors
        // Example: public DcMotor randomMotor;

    // Drive Speed & Boost Speed (When Holding Left Trigger)
    public double driveSpeed = 0.4;
    public double boostSpeed = 0.8;

    /// ROBOT CONSTRUCTOR ///
    public Robot(HardwareMap hardwareMap) {
        // IMU (For Field Centric Drive)
        imu = hardwareMap.get(IMU.class, "imu");

        // Drive Train Motors
        this.frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        this.backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        this.frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        this.backRightMotor = hardwareMap.dcMotor.get("rightRear");

        // Setting Drive Train Motor Direction
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Other Motors (Example)
        //this.randomMotor = hardwareMap.dcMotor.get("random motor");

        // Adjust Orientation Parameters For IMU And Field Centric Drive
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);
    }

    // Field Centric Drive
    public void drive(double leftx, double lefty, double rightx, double lt, boolean resetYaw) {
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
        rotX = rotX * 1.1;

        // Drive Train Calculations
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Final Speed
        double finalSpeed = lerp(this.driveSpeed, this.boostSpeed, lt);

        // Apply Power
        frontLeftMotor.setPower(frontLeftPower * (finalSpeed + lt * finalSpeed));
        backLeftMotor.setPower(backLeftPower * (finalSpeed + lt * finalSpeed));
        frontRightMotor.setPower(frontRightPower * (finalSpeed + lt * finalSpeed));
        backRightMotor.setPower(backRightPower * (finalSpeed + lt * finalSpeed));
    }

    // Finds T Of The Way From Value A To Value B
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
