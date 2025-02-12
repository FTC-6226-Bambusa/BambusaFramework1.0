package Bambusa;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Constructor 1
    public PID(String motorName) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        this.p = 0;
        this.i = 0;
        this.d = 0;
        this.f = 0;
    }

    // Constructor 2
    public PID(String motorName, double p, double i, double d, double f) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    // Constructor 3
    public PID(String motorName, double p, double i, double d, double f, double s, double v, double a) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        // PID Values
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    // Moves Motor To Target
    public void moveTo(double target) {
        this.motor.setPower(calculatePower(target, this.motor.getCurrentPosition()));
    }

    // Calculates Power Output
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

    // Sets PID Values
    public void setPID(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
}
