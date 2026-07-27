package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


public class SwervePod {
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final AnalogInput absoluteEncoder;
    private final double encoderOffset;
    private double kP = 0.0; //0.007
    private double kI = 0.0; //0.0
    private double kD = 0.0; //0.00005
    private double kS = 0.0; //0.005

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    private double TICKS = 4096;

    public SwervePod(DcMotorEx motor1, DcMotorEx motor2, AnalogInput absoluteEncoder, double encoderOffset) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.absoluteEncoder = absoluteEncoder;
        this.encoderOffset = encoderOffset;

        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getModuleAngle() {
        double voltage = absoluteEncoder.getVoltage();
        return (voltage / 3.2) * 2.0 * Math.PI;
    }
    public void setPID(double p, double i, double d, double s) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kS = s;
    }
    public void update (double targetAngle, double drivePower, double currentTime) {
        double currentVoltage = motor2.getCurrentPosition();
        double currentAngle = (currentVoltage / TICKS) * 120 - (encoderOffset);

        double error = targetAngle - currentAngle;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
//
//        pidf.setPIDF(kP, kI, kD, kS);
//        double steeringPower = pidf.calculate(-error, 0);

        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.01;

        // 4. Integral term calculation with anti-windup clamping
        integralSum += error * dt;
        if (kI != 0) {
            integralSum = Range.clip(integralSum, -0.2 / kI, 0.2 / kI);
        }

        // 5. Derivative term calculation
        double derivative = (error - lastError) / dt;

        // 6. Directional Static Friction Feedforward (kS)
        double feedforward = 0.0;
        if (Math.abs(error) > 0.5) { // 0.5-degree deadband tolerance
            feedforward = Math.signum(error) * kS;
        }

        // Save states for next loop iteration
        lastError = error;
        lastTime = currentTime;

        double steeringPower = (error * kP) + (integralSum * kI) + (derivative * kD) + feedforward;
        steeringPower = Range.clip(steeringPower, -1.0, 1.0);
        steeringPower = Range.clip(steeringPower, -1.0, 1.0);

        // 5. Cosine optimization to prevent wheel scrubbing during sharp turns
        double optimizedDrivePower = drivePower * Math.cos(Math.toRadians(error));

        double m1Power = steeringPower + optimizedDrivePower;
        double m2Power = steeringPower - optimizedDrivePower;

        double max = Math.max(Math.abs(m1Power), Math.max(Math.abs(m2Power), 1.0));
        motor1.setPower(m1Power / max);
        motor2.setPower(m2Power / max);
    }
}
