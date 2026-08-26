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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.controller.PIDFController;
import com.bylazar.telemetry.PanelsTelemetry;

@Configurable
@TeleOp(name = "DiffySwervePodTunerA", group = "testing")
public class SwerveModulePIDTestingA extends OpMode {
    private DcMotorEx motor1, motor2;
    AnalogInput absoluteEncoder;
    public static double targetAngle = 0.0;
    public static double encoderOffset = 0.0;
    public static double drive = 0.0;
    public static double maxMotorVelocity = 1800;
    public static double maxSteeringVelocity = 500;
    private double driveDirectionSign = 1.0;
    public static double deadband = 2.0;
    public static double ratio = -1.0;
    public static double kP = 5.85;
    public static double kI = 0.0; 
    public static double kD = 0.038; //0.0001
    public static double kS = 220;
    private double m1P = 4.3;
    private double m1I = 0.0;
    private double m1D = 0.0;
    private double m1F = 6.0;
    private double m2P = 15.0;
    private double m2I = 0.0;
    private double m2D = 1.0;
    private double m2F = 13.0;

    // Added tracking variables for manual I and D loops
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //right now, both positive = turn, so to move forward one must be positive and other must be negative :)

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "encoderA");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //lastTime = getRuntime();
        telemetryM.addLine("Testing");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m1P, m1I, m1D, m1F));
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m2P, m2I, m2D, m2F));

        double currentVoltage = absoluteEncoder.getVoltage();;
        double currentAngle = (currentVoltage / 3.2) * 360 - (encoderOffset);

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
        if (Math.abs(error) > deadband) { //2 deg of tolerance
            feedforward = Math.signum(error) * kS;
        }

        // Save states for next loop iteration
        lastError = error;
        lastTime = currentTime;

        // 7. Compute total output power (PID + F)
        double steeringVelocity = (error * kP) + (integralSum * kI) + (derivative * kD) + feedforward;
        steeringVelocity = Range.clip(steeringVelocity, -maxSteeringVelocity, maxSteeringVelocity);

        double optimizedDriveVelocity = (drive * driveDirectionSign * maxMotorVelocity) * Math.cos(Math.toRadians(error));

        double targetVel1 = ratio * steeringVelocity + optimizedDriveVelocity;
        double targetVel2 = steeringVelocity + optimizedDriveVelocity;

        double max = Math.max(Math.abs(targetVel1), Math.max(Math.abs(targetVel2), maxMotorVelocity));
        motor1.setVelocity((targetVel1 / max) * maxMotorVelocity);
        motor2.setVelocity((targetVel2 / max) * maxMotorVelocity);



        telemetryM.addData("target_angle_deg", targetAngle);
        telemetryM.addData("current_angle_deg", (int)currentAngle);
        telemetryM.addData("target 1", (int)targetVel1);
        telemetryM.addData("target 2", (int)targetVel2);
        telemetryM.addData("error_deg", error);
        telemetryM.addData("steering_power_output", steeringVelocity);
        telemetryM.update(telemetry);

    }
}