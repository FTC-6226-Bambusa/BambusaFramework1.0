package Bambusa;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is a sample PID for Bambusa 6226
 * This class is designed to be able to control a single motor, and be able to move it to a target quickly and accurately.
 * Here are what each of the values mean:
 *
 * Main Values:
 * P: Proportion - How fast you want to get to your target, depending on the error.
 * I: Integral - How much faster you want to go the longer it takes to get to your target.
 * D: Derivative - How much you dampen the force so that the motor does not overshoot the target.
 * F: Feedforward - How much you want the motor to counteract gravity.
 *
 * Advanced Values:
 * S: Static Friction - How much power you want to use to overcome friction.
 * A: Acceleration Gain - How much you want to accelerate to counter inertia.
 * V: Velocity Gain - How much you want to counter inertia.
 *
 * Ticks Per Degree: The number of ticks added to the position after the motor rotates one degree.
 * Horizontal Pos: Used for the feedforward - tick position of arm when it is level to the ground.
*/



public class PID {
    // Motor Which Has PID
    public DcMotorEx motor;

    // Timer
    public ElapsedTime timer = new ElapsedTime();

    // PID Values (With Feedforward)
    public double p, i, d, f, v, a, s;
    public double horizontalPos;
    public double ticksPerDegree = 90.0 / 360;

    private double integralSum = 0;
    private double lastPower = 0;
    private double lastError = 0;
    private double lastPos = 0;
    private double lastVel = 0;

    /// CONSTRUCTORS ///
    public PID(DcMotorEx motor) {
        this.motor = motor;

        this.horizontalPos = motor.getCurrentPosition();

        this.p = 0;
        this.i = 0;
        this.d = 0;
        this.f = 0;

        this.s = 0;
        this.a = 0;
        this.v = 0;
    }

    public PID(DcMotorEx motor, double p, double i, double d, double f) {
        this.motor = motor;

        this.horizontalPos = motor.getCurrentPosition();

        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        this.s = 0;
        this.a = 0;
        this.v = 0;
    }

    public PID(DcMotorEx motor, double p, double i, double d, double f, double tpd) {
        this.motor = motor;

        this.ticksPerDegree = tpd;
        this.horizontalPos = motor.getCurrentPosition();

        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        this.s = 0;
        this.a = 0;
        this.v = 0;
    }

    public PID(DcMotorEx motor, double p, double i, double d, double f, double s, double a, double v) {
        this.motor = motor;

        this.horizontalPos = motor.getCurrentPosition();

        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        this.s = s;
        this.a = a;
        this.v = v;
    }

    public PID(DcMotorEx motor, double p, double i, double d, double f, double s, double a, double v, double tpd) {
        this.motor = motor;

        this.ticksPerDegree = tpd;
        this.horizontalPos = motor.getCurrentPosition();

        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        this.s = s;
        this.a = a;
        this.v = v;
    }

    // Calculates Power Output - MAIN PID MATH
    public double calculatePower(double target) {
        double pos = this.motor.getCurrentPosition();
        double error = target - pos;
        double derivative = (error - lastError) / timer.milliseconds() * 1000;
        double feed = Math.abs(Math.cos(Math.toRadians((pos - this.horizontalPos) / ticksPerDegree)));

        double velocity = (pos - lastPos) / timer.milliseconds() * 1000;
        double acceleration = (velocity - lastVel) / timer.milliseconds() * 1000;
        double direction = Math.abs(error) / (error == 0 ? 1 : error);

        double output = (error * this.p) + (derivative * this.d) + (integralSum * this.i) + (feed * this.f) +
                        (velocity * this.v) + (acceleration * this.a) + (direction * this.s);

        integralSum += error * timer.seconds();

        lastPos = pos;
        lastVel = velocity;
        lastError = error;

        timer.reset();

        return output;
    }

    // Floats Using Feedforward Constant - Good For Tuning Feeforward
    public void levitate() {
        double pos = this.motor.getCurrentPosition();
        double feed = Math.abs(Math.cos(Math.toRadians((pos - this.horizontalPos) / ticksPerDegree)));

        this.motor.setPower(feed);
    }

    // Disables PID
    public void disable() {
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor.setPower(0);
    }

    // Tries To Stay Still Without Using Feedforward Constant
    public void brake() {
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setPower(0);
    }

    // Moves Motor To Target
    public void moveTo(double target) {
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.lastPower = calculatePower(target);
        this.motor.setPower(lastPower);
    }

    // Sets Motor Power
    public void setPower(double power) {
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor.setPower(power);

        this.lastPower = power;
    }

    // Sets PID Values
    public void setPID(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    // Sets SAV Values
    public void setSAV(double s, double a, double v) {
        this.s = s;
        this.a = a;
        this.v = v;
    }

    // Sets Horizontal Position (For Feedforward)
    public void setHorizontalPos(double pos) {
        this.horizontalPos = pos;
    }

    // Sets Ticks Per Degree (For Feedforward)
    public void setTicksPerDegree(double tpd) {
        this.ticksPerDegree = tpd;
    }

    // Helps User Find Correct Target Position (If FTC Dashboard Is Not Available)
    public double testPosition(double speed, boolean moveForward, boolean moveBackward) {
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setPower(moveForward ? speed : (moveBackward ? -speed : 0));

        return this.motor.getCurrentPosition();
    }

    // Returns Error
    public double getError() {
        return this.lastError;
    }

    // Returns Velocity
    public double getVelocity() {
        return this.lastVel;
    }

    // Returns Last Power Output
    public double getPower() {
        return this.lastPower;
    }

    // Gets Current Position Of Motor
    public double getPos() {
        return this.motor.getCurrentPosition();
    }

    // Calculates Degrees Per Tick
    public double degreesToTicks(double degrees) {
        return degrees / this.ticksPerDegree;
    }

    // Determines Whether Arm Is At Target
    public boolean isAtTarget(double maxError, double maxSpeed) {
        return Math.abs(this.lastError) < maxError && Math.abs(this.lastVel) < maxSpeed;
    }
}