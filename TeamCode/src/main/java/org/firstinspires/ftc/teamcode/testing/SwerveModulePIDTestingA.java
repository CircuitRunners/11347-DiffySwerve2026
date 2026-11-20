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
    public static double encoderOffset = 101.0;
    public static double drive = 0.0;
    public static double maxMotorVelocity = 2375;
    public static double maxSteeringVelocity = 260;
    private double driveDirectionSign = 1.0;
    public static double deadband = 2.0;
    public static double kP = 4.0; //4
    public static double kI = 0.0; 
    public static double kD = 0.00001; //0.00001
    public static double kS = 5.0; //5
    private double m1P = 15.0;
    private double m1I = 1.0;
    private double m1D = 0.0;
    private double m1F = 13.6;
    private double m2P = 15.0;
    private double m2I = 1.5;
    private double m2D = 0.0;
    private double m2F = 13.6;

    // Added tracking variables for manual I and D loops
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    public static double driveRampRate = 1;// power per second
    private double currentDrivePower = 0.0;
    private double lastDriveTime = 0.0;
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
        lastTime = getRuntime();
        lastDriveTime = getRuntime();
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
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.01;

        // 4. Integral term calculation with anti-windup clamping
        integralSum += optimizedError * dt;
        if (kI != 0) {
            integralSum = Range.clip(integralSum, -0.2 / kI, 0.2 / kI);
        }

        // 5. Derivative term calculation
        double derivative = (optimizedError - lastError) / dt;

        // 6. Directional Static Friction Feedforward (kS)
        double feedforward = 0.0;
        if (Math.abs(optimizedError) > deadband) { //2 deg of tolerance
            feedforward = Math.signum(optimizedError) * kS;
        }

        // Save states for next loop iteration
        lastError = optimizedError;
        lastTime = currentTime;

        // 7. Compute total output power (PID + F)
        double steeringVelocity = (optimizedError * kP) + (integralSum * kI) + (derivative * kD) + feedforward;
        steeringVelocity = Range.clip(steeringVelocity, -maxSteeringVelocity, maxSteeringVelocity);

        currentTime = getRuntime();
        double driveDt = currentTime - lastDriveTime;

        if (driveDt <= 0) {
            driveDt = 0.01;
        }

        double maxDriveChange = driveRampRate * driveDt;

// Move currentDrivePower toward drive
        if (currentDrivePower < drive) {
            currentDrivePower += maxDriveChange;
            if (currentDrivePower > drive) {
                currentDrivePower = drive;
            }
        } else if (currentDrivePower > drive) {
            currentDrivePower -= maxDriveChange;
            if (currentDrivePower < drive) {
                currentDrivePower = drive;
            }
        }

        lastDriveTime = currentTime;

        double optimizedDriveVelocity = currentDrivePower * driveDirectionSign * maxMotorVelocity;

        double targetVel1 = steeringVelocity - optimizedDriveVelocity;
        double targetVel2 = steeringVelocity + optimizedDriveVelocity;

        double max = Math.max(Math.abs(targetVel1), Math.max(Math.abs(targetVel2), maxMotorVelocity));
        motor1.setVelocity((targetVel1 / max) * maxMotorVelocity);
        motor2.setVelocity((targetVel2 / max) * maxMotorVelocity);



        telemetryM.addData("target_angle_deg", targetAngle);
        telemetryM.addData("current_angle_deg", (int)currentAngle);
        telemetryM.addData("target 1", (int)targetVel1);
        telemetryM.addData("target 2", (int)targetVel2);
        telemetryM.addData("motor 1 velocity:", motor1.getVelocity());
        telemetryM.addData("motor 2 velocity:", motor2.getVelocity());
        telemetryM.addData("error_deg", error);
        telemetryM.addData("steering_power_output", steeringVelocity);
        telemetryM.update(telemetry);

    }
}