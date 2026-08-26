package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;


public class SwervePod {
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final AnalogInput absoluteEncoder;
    private final double encoderOffset;
    //Rotation PID
    private double kP;
    private double kI;
    private double kD;
    private double kS;
    //Motor PID
    private double ratio;
    private final double maxMotorVelocity = 1800;
    private final double maxSteeringVelocity = 500;
    private final double deadband = 2;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double driveDirectionSign = 1.0;

    private double targetAngle;
    private double drivePower;
    public SwervePod(DcMotorEx motor1, DcMotorEx motor2, AnalogInput absoluteEncoder, double encoderOffset,
                     double kP, double kI, double kD, double kS,
                     double m1P, double m1I, double m1D, double m1F,
                     double m2P, double m2I, double m2D, double m2F,
                     double ratio) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.absoluteEncoder = absoluteEncoder;

        this.encoderOffset = encoderOffset;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;

        this.ratio = ratio;

        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m1P, m1I, m1D, m1F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m2P, m2I, m2D, m2F));
        lastTime = getTime();
    }

    public void setTarget(double angle, double speed) {

        targetAngle = angle;

        drivePower = Range.clip(
                speed,
                -1.0,
                1.0
        );
    }

    public void update () {
        double currentAngle = getAngle();
        double error = targetAngle - currentAngle;

        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        driveDirectionSign = 1.0;

        if (error > 90) {
            error -= 180;
            driveDirectionSign = -1.0;
        } else if (error < -90) {
            error += 180;
            driveDirectionSign = -1.0;
        }

        drivePower *= driveDirectionSign;

        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.01;
        }

        integralSum += error * dt;

        if (kI != 0) {
            integralSum = Range.clip(integralSum, -0.2 / kI, 0.2 / kI);
        }

        double derivative = (error - lastError) / dt;

        double feedforward = 0;

        if (Math.abs(error) > deadband) {
            feedforward = Math.signum(error) * kS;
        }

        double steeringVelocity = error * kP + integralSum * kI + derivative * kD + feedforward;
        steeringVelocity = Range.clip(steeringVelocity, -maxSteeringVelocity, maxSteeringVelocity);

        lastError = error;
        lastTime = currentTime;

        double optimizedDriveVelocity = (drivePower * driveDirectionSign * maxMotorVelocity) * Math.cos(Math.toRadians(error));
        double targetVel1 = ratio * steeringVelocity - ratio * optimizedDriveVelocity;
        double targetVel2 = steeringVelocity + optimizedDriveVelocity;

        double max = Math.max(Math.abs(targetVel1), Math.max(Math.abs(targetVel2), maxMotorVelocity));
        motor1.setVelocity((targetVel1 / max) * maxMotorVelocity);
        motor2.setVelocity((targetVel2 / max) * maxMotorVelocity);
    }
    public double getAngle() {

        double currentVoltage =
                absoluteEncoder.getVoltage();

        return (currentVoltage / 3.2) * 360.0
                - encoderOffset;
    }
    public double getSteeringError(double targetAngle) {

        double error = targetAngle - getAngle();

        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    public void stop() {

        motor1.setVelocity(0);
        motor2.setVelocity(0);
    }

    private double getTime() {

        return System.nanoTime() / 1e9;
    }
}
