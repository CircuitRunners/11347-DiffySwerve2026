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
    private double kS1;
    private double kS2;
    private double kS3;
    private double kS = 30;
    //Motor PID
    private double ratio;
    private final double maxMotorVelocity = 2375;
    private final double maxSteeringVelocity = 260;
    private final double deadband = 2;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double driveDirectionSign = 1.0;

    private double targetAngle;
    private double drivePower;
    private double optimizedTarget;
    private double driveRampRate = 0.9; // power per second
    private double currentDrivePower = 0.0;
    private double lastDriveTime = 0.0;
    private boolean changingAngle = false;
    private double lastTargetAngle = 0.0;
    private final double angleChangeThreshold = 5.0;
    public SwervePod(DcMotorEx motor1, DcMotorEx motor2, AnalogInput absoluteEncoder, double encoderOffset,
                     double kP, double kI, double kD, double kS1, double ks2, double ks3,
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
        this.kS1 = kS1;
        this.kS2 = kS2;
        this.kS3 = kS3;

        this.ratio = ratio;

        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m1P, m1I, m1D, m1F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m2P, m2I, m2D, m2F));
        lastTime = getTime();
        lastDriveTime = getTime();
    }

    public void setTarget(double angle, double speed) {

        targetAngle = angle;

        drivePower = Range.clip(
                speed,
                -1.0,
                1.0
        );
        if (drivePower < 0.3) {
            kS = kS1;
        }
        else if (drivePower >=0.3 && drivePower <=0.6) {
            kS = kS2;
        }
        else {
            kS = kS3;
        }
    }

    public void update () {
        if (Math.abs(drivePower) < 0.001) {
            currentDrivePower = 0.0;

            motor1.setVelocity(0);
            motor2.setVelocity(0);

            return;
        }



        double currentAngle = getAngle();
        double error = targetAngle - currentAngle;


        //wraps error to be between -180 - 180
        while (error > 180)
            error -= 360;

        while (error < -180)
            error += 360;

        optimizedTarget = targetAngle;
        driveDirectionSign = 1.0;

        if (error > 90) {
            optimizedTarget = targetAngle - 180;
            driveDirectionSign = -1.0;
        } else if (error < -90) {
            optimizedTarget = targetAngle + 180;
            driveDirectionSign = -1.0;
        }

        double optimizedError = error;

        if (error > 90) {
            optimizedError = error - 180;
            driveDirectionSign = -1.0;
        } else if (error < -90) {
            optimizedError = error + 180;
            driveDirectionSign = -1.0;
        } else {
            driveDirectionSign = 1.0;
        }

        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.01;
        }

        integralSum += optimizedError * dt;

        if (kI != 0) {
            integralSum = Range.clip(integralSum, -0.2 / kI, 0.2 / kI);
        }

        double derivative = (optimizedError - lastError) / dt;

        double feedforward = 0;

        if (Math.abs(optimizedError) > deadband) {
            feedforward = Math.signum(optimizedError) * kS;
        }

        double steeringVelocity = optimizedError * kP + integralSum * kI + derivative * kD + feedforward;
        steeringVelocity = Range.clip(steeringVelocity, -maxSteeringVelocity, maxSteeringVelocity);

        lastError = optimizedError;
        lastTime = currentTime;

        currentTime = getTime();
        double driveDt = currentTime - lastDriveTime;

        if (driveDt <= 0) {
            driveDt = 0.01;
        }

        double maxDriveChange = driveRampRate * driveDt;

// Move currentDrivePower toward drive
        if (currentDrivePower < drivePower) {
            currentDrivePower += maxDriveChange;
            if (currentDrivePower > drivePower) {
                currentDrivePower = drivePower;
            }
        } else if (currentDrivePower > drivePower) {
            currentDrivePower -= maxDriveChange;
            if (currentDrivePower < drivePower) {
                currentDrivePower = drivePower;
            }
        }

        lastDriveTime = currentTime;

        double optimizedDriveVelocity =
                currentDrivePower * driveDirectionSign * maxMotorVelocity;

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
