package Bambusa;

import static Bambusa.MathPlus.AngleWrap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import RoadRunner.MecanumDrive;

public class PurePursuitAuto {
    public MecanumDrive drivetrain;
    public IMU imu;
    public DcMotor topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;

    public PurePursuitAuto(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        topLeftMotor = hardwareMap.dcMotor.get("TOP_LEFT");
        bottomRightMotor = hardwareMap.dcMotor.get("TOP_LEFT");
        bottomLeftMotor = hardwareMap.dcMotor.get("TOP_LEFT");
        topRightMotor = hardwareMap.dcMotor.get("TOP_LEFT");
    }

    public void navigate() {
        Pose2d pose = drivetrain.localizer.getPose();

        Vector2d position = pose.position;
        Rotation2d rotation = pose.heading;

    }

    public void goToPosition(double x, double y, double speed) {
        Pose2d pose = drivetrain.localizer.getPose();
        Vector2d pos = pose.position;
        Rotation2d rot = pose.heading;

        double dist = Math.hypot(x - pos.x, y - pos.y);
        double absoluteAngle = Math.atan2(y - pos.y, x - pos.x);
        double relativeAngle = AngleWrap(absoluteAngle - (rot.real - Math.toRadians(90)));

        double relativeX = Math.cos(relativeAngle) * dist;
        double relativeY = Math.sin(relativeAngle) * dist;

        double movementX = relativeX * speed;
        double movementY = relativeY * speed;

        /// Calculating Mecanum Wheel Motor Powers///

        double power_y = -movementY;
        double power_x = -movementX * 1.1;
        double rx = 0; // Currently no rotation added

        // Rotation Of Robot
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotates Movement Direction To Counter Bot Rotation
        double rotX = power_x * Math.cos(botHeading) - power_y * Math.sin(botHeading);
        double rotY = power_x * Math.sin(botHeading) + power_y * Math.cos(botHeading);

        // Counters Imperfect Strafing
        rotX *= 1.1;

        // Calculations
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Apply Power
        topLeftMotor.setPower(frontLeftPower * speed);
        bottomLeftMotor.setPower(backLeftPower * speed);
        topRightMotor.setPower(frontRightPower * speed);
        bottomRightMotor.setPower(backRightPower * speed);
    }
}
