package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.pedropathing.control.PIDFController;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.controller.PIDFController;
import com.bylazar.telemetry.PanelsTelemetry;

@Configurable
@TeleOp(name = "DiffySwervePodTuner", group = "testing")
public class SwerveModulePIDTesting extends OpMode {
    private DcMotorEx motor1, motor2;
    //private DigitalChannel absoluteEncoder;

    public static double targetAngle = 0.0;
    public static double encoderOffset = 0.0;
    public static double drivePower = 0.0;
    private double driveDirectionSign = 1.0;
    public static double TICKS = 4096;

    public static double kP = 0.004;
    public static double kI = 0.0; 
    public static double kD = 0.00018; //0.0001
    public static double kS = 0.08;

    // Added tracking variables for manual I and D loops
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void init() {

        // 1. Map your single pod's hardware
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        //absoluteEncoder = hardwareMap.get(DigitalChannel.class, "podEncoder");

        // 2. Enforce BRAKE mode and RAW power control
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //lastTime = getRuntime();
        telemetryM.addLine("Testing");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        //pidf.setSetPoint(targetAngle);

        double currentVoltage = motor2.getCurrentPosition();
        double currentAngle = (currentVoltage / TICKS) * 120 - (encoderOffset);

        double error = targetAngle - currentAngle;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;


        if (error > 90) {
            error -= 180;
            driveDirectionSign = -1.0;
        } else if (error < -90) {
            error += 180;
            driveDirectionSign = -1.0;
        }

        double currentTime = getRuntime();
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

        // 7. Compute total output power (PID + F)
        double steeringPower = (error * kP) + (integralSum * kI) + (derivative * kD) + feedforward;
        steeringPower = Range.clip(steeringPower, -1.0, 1.0);
        steeringPower = Range.clip(steeringPower, -1.0, 1.0);

        double optimizedDrivePower = (drivePower * driveDirectionSign) * Math.cos(Math.toRadians(error));

        double m1Power = steeringPower + optimizedDrivePower;
        double m2Power = steeringPower - optimizedDrivePower;

        double max = Math.max(Math.abs(m1Power), Math.max(Math.abs(m2Power), 1.0));
        motor1.setPower(m1Power / max);
        motor2.setPower(m2Power / max);

        telemetryM.addData("target_angle_deg", targetAngle);
        telemetryM.addData("current_angle_deg", (int)currentAngle);
        telemetryM.addData("error_deg", error);
        telemetryM.addData("steering_power_output", steeringPower);
        telemetryM.update(telemetry);

    }
}