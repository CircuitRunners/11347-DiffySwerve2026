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
@TeleOp(name = "DiffySwervePodTuner", group = "testing")
public class SwerveModulePIDTesting extends OpMode {
    private DcMotorEx motor1, motor2;
    AnalogInput absoluteEncoder;
    public static double targetAngle = 0.0;
    public static double encoderOffset = 0.0;
    public static double driveVelocity = 0.0;
    public static double maxMotorVelocity = 1800;
    private double driveDirectionSign = 1.0;
    public static double TICKS = 4096;
    public static double ratio = -1.0;
    public static double kP = 0.004;
    public static double kI = 0.0; 
    public static double kD = 0.00018; //0.0001
    public static double kS = 0.08;
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
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "podEncoder");

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
        if (Math.abs(error) > 2) { //2 deg of tolerance
            feedforward = Math.signum(error) * kS;
        }

        // Save states for next loop iteration
        lastError = error;
        lastTime = currentTime;

        // 7. Compute total output power (PID + F)
        double steeringVelocity = (error * kP) + (integralSum * kI) + (derivative * kD) + feedforward;
        steeringVelocity = Range.clip(steeringVelocity, -maxMotorVelocity, maxMotorVelocity);

        double optimizedDriveVelocity = (driveVelocity * driveDirectionSign * maxMotorVelocity) * Math.cos(Math.toRadians(error));

        double targetVel1 = ratio * steeringVelocity + optimizedDriveVelocity;
        double targetVel2 = ratio * steeringVelocity - optimizedDriveVelocity;

        double max = Math.max(Math.abs(targetVel1), Math.max(Math.abs(targetVel2), maxMotorVelocity));
        motor1.setVelocity((targetVel1 / max) * maxMotorVelocity);
        motor2.setVelocity((targetVel2 / max) * maxMotorVelocity);



        telemetryM.addData("target_angle_deg", targetAngle);
        telemetryM.addData("current_angle_deg", (int)currentAngle);
        telemetryM.addData("error_deg", error);
        telemetryM.addData("steering_power_output", steeringVelocity);
        telemetryM.update(telemetry);

    }
}

//package org.firstinspires.ftc.teamcode.testing;
//
//import com.bylazar.configurables.annotations.Configurable;
////import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
////import com.seattlesolvers.solverslib.command.CommandOpMode;
////import com.pedropathing.control.PIDFController;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.util.Range;
////import com.seattlesolvers.solverslib.controller.PIDFController;
//import com.bylazar.telemetry.PanelsTelemetry;
//
//@Configurable
//@TeleOp(name = "DiffySwervePodTuner", group = "testing")
//public class SwerveModulePIDTesting extends OpMode {
//    private DcMotorEx motor1, motor2;
//    private AnalogInput absoluteEncoder;
//    public static double targetAngle = 0.0;
//    public static double encoderOffset = 0.0; //Right side upside down!: idk i forgot it
//    public static double drivePower = 0.0;
//    public static double driveDirectionSign = 1.0;
//    public static double ratio = 1;
//
//    public static double kP = 0.007; //0.004  //0.0035 //Right side: 0.007
//    public static double kI = 0.0; //Right side: 0.00
//    public static double kD = 0.00005; //0.00018  //0.00002 //Right Side: 0.00005
//    public static double kS = 0.005; //0.08  //0.07 //Right side: 0.005
//
//    // Added tracking variables for manual I and D loops
//    private double integralSum = 0.0;
//    private double lastAngle = 0.0;
//    private double lastTime = 0.0;
//    @IgnoreConfigurable
//    static TelemetryManager telemetryM;
//
//    @Override
//    public void init() {
//
//        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
//        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
//        absoluteEncoder = hardwareMap.get(AnalogInput.class, "podEncoder");
//
//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        //lastTime = getRuntime();
//        telemetryM.addLine("Testing");
//        telemetryM.update(telemetry);
//    }
//    @Override
//    public void start() {
//        lastTime = getRuntime();
//        double initialVoltage = absoluteEncoder.getVoltage();
//        lastAngle = (initialVoltage / 3.3) * 360.0 * ratio - encoderOffset;
//    }
//
//    @Override
//    public void loop() {
//        //pidf.setSetPoint(targetAngle);
//
//        double currentVoltage = absoluteEncoder.getVoltage();
//        double currentAngle = (currentVoltage / 3.2) * 360.0 * ratio - (encoderOffset);
//
//        double error = targetAngle - currentAngle;
//        while (error > 180) error -= 360;
//        while (error < -180) error += 360;
//
//        driveDirectionSign = 1.0;
//        if (error > 90) {
//            error -= 180;
//            driveDirectionSign = -1.0;
//        } else if (error < -90) {
//            error += 180;
//            driveDirectionSign = -1.0;
//        }
//
//        double currentTime = getRuntime();
//        double dt = currentTime - lastTime;
//        if (dt <= 0) dt = 0.01;
//
//        // 4. Integral term calculation with anti-windup clamping
//        integralSum += error * dt;
//        if (kI != 0) {
//            integralSum = Range.clip(integralSum, -0.2 / kI, 0.2 / kI);
//        }
//
//        // 5. Derivative term calculation
//        double angleDifference = currentAngle - lastAngle;
//        while (angleDifference > 180) angleDifference -= 360;
//        while (angleDifference < -180) angleDifference += 360;
//
//        double derivative = -angleDifference / dt;
//
//        // 6. Directional Static Friction Feedforward (kS)
//        double feedforward = 0.0;
//        if (Math.abs(error) > 0.5) { // 0.5-degree deadband tolerance
//            feedforward = Math.signum(error) * kS;
//        }
//
//        // Save states for next loop iteration
//        lastAngle = currentAngle;
//        lastTime = currentTime;
//
//        // 7. Compute total output power (PID + F)
//        double steeringPower = (error * kP) + (integralSum * kI) + (derivative * kD) + feedforward;
//        steeringPower = Range.clip(steeringPower, -1.0, 1.0);
//        steeringPower = Range.clip(steeringPower, -1.0, 1.0);
//
//        double optimizedDrivePower = (drivePower * driveDirectionSign) * Math.cos(Math.toRadians(error));
//
//        double m1Power = -steeringPower + optimizedDrivePower;
//        double m2Power = -steeringPower - optimizedDrivePower;
//
//        double max = Math.max(Math.abs(m1Power), Math.max(Math.abs(m2Power), 1.0));
//        motor1.setPower(m1Power / max);
//        motor2.setPower(m2Power / max);
//
//
//        telemetryM.addData("Raw Encoder", absoluteEncoder.getVoltage());
//        telemetryM.addData("target_angle_deg", targetAngle);
//        telemetryM.addData("current_angle_deg", (int)currentAngle);
//        telemetryM.addData("error_deg", error);
//        telemetryM.addData("steering_power_output", steeringPower);
//        telemetryM.update(telemetry);
//
//    }
//}