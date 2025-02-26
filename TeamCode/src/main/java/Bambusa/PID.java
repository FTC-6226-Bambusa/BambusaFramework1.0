package Bambusa;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: Add kS, kA, and kV values for an advanced PID.

/*
 * This is a sample PID for Bambusa 6226
 *
 * This class is designed to be able to control a single motor, with some feedback.
 *
 * Here are what each of the values (P, I, D, and F) mean:
 *
 * P: Proportion - How fast you want to get to your target, depending on the error.
 * I: Integral - How much faster you want to go the longer it takes to get to your target.
 * D: Derivative - How much you dampen the force so that the motor does not overshoot the target.
 * F: Feedforward - How much you want the motor to counteract gravity.
 */



public class PID {
    // Motor Which Has PID
    DcMotorEx motor;

    // Timer
    ElapsedTime timer = new ElapsedTime();

    // PID Values (With Feedforward)
    private double p, i, d, f;

    private double integralSum = 0;
    private double lastError = 0;
    private double ticksPerDegree = 90.0 / 360;

    /// CONSTRUCTORS ///
    public PID(DcMotorEx motor) {
        this.motor = motor;

        this.p = 0;
        this.i = 0;
        this.d = 0;
        this.f = 0;
    }

    public PID(DcMotorEx motor, double p, double i, double d, double f) {
        this.motor = motor;

        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public PID(DcMotorEx motor, double tpd, double p, double i, double d, double f) {
        this.motor = motor;

        this.ticksPerDegree = tpd;

        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    // Moves Motor To Target
    public void moveTo(double target) {
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor.setPower(calculatePower(target, this.motor.getCurrentPosition()));
    }

    // Calculates Power Output - MAIN PID MATH
    public double calculatePower(double target, double pos) {
        double error = target - pos;
        double derivative = (error - lastError) / timer.milliseconds() * 1000;
        double feed = Math.cos(Math.toRadians(pos / ticksPerDegree));

        integralSum += error * timer.milliseconds() / 1000;
        lastError = error;

        double output = (error * this.p) + (derivative * this.d) + (integralSum * this.i) + (feed * this.f);

        timer.reset();

        return output;
    }

    // Helps User Find Correct Target Position
    public double testPosition(double speed, boolean moveForward, boolean moveBackward) {
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setPower(moveForward ? speed : (moveBackward ? -speed : 0));

        return this.motor.getCurrentPosition();
    }

    // Sets PID Values
    public void setPID(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    // Sets Ticks Per Degree (For Feedforward)
    public void setTicksPerDegree(double tpd) {
        this.ticksPerDegree = tpd;
    }

    // Returns Error
    public double getError() {
        return this.lastError;
    }
}
