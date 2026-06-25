package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveModule {
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final AnalogInput absoluteEncoder;

    private final double encoderOffset;

    public SwerveModule(DcMotorEx motor1, DcMotorEx motor2, AnalogInput absoluteEncoder, double encoderOffset) {
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

    public void setTargetAngle(double targetAngleRadians) {
        double currentAngle = getModuleAngle() - encoderOffset;

        double error = targetAngleRadians - currentAngle;
        while (error > Math.PI) error -= 2.0 * Math.PI;
        while (error < -Math.PI) error += 2.0 * Math.PI;

        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) deltaTime = 0.001;

        integralSum += error * deltaTime;
        integralSum = Math.max(-0.5, Math.min(0.5, integralSum));

        double derivative = (error - lastError) / deltaTime;
        double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);

        double staticFeedforward = 0.0;
        if (Math.abs(error) > Math.toRadians(1.0)) {
            staticFeedforward = Math.signum(error) * kS;
        }

        double steeringPower = pidOutput + staticFeedforward;
        steeringPower = Math.max(-1.0, Math.min(1.0, steeringPower));
        lastError = error;
        lastTime = currentTime;

        double motor1Power = driveSpeed + steeringPower;
        double motor2Power = driveSpeed - steeringPower;

        double maxPower = Math.max(1.0, Math.max(Math.abs(motor1Power), Math.abs(motor2Power)));
        
        motor1.setPower(motor1Power / maxPower);
        motor2.setPower(motor2Power / maxPower);
    }
}
}