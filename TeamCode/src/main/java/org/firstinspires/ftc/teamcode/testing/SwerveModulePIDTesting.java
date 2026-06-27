package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Configurable
@TeleOp(name = "DiffySwervePodTuner", group = "testing")
public class SwerveModulePIDTesting extends OpMode {
    private DcMotorEx motor1, motor2;
    private AnalogInput absoluteEncoder;

    public static double targetAngle = 0.0;
    public static double encoderOffset = 270.0;

    public static double kP = 0.0; 
    public static double kI = 0.0; 
    public static double kD = 0.0; 
    public static double kS = 0.0;

    private PIDFController pidf;

    @Override
    public void init() {

        // 1. Map your single pod's hardware
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "podEncoder");

        // 2. Enforce BRAKE mode and RAW power control
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(kP, kI, kD, kS);
        pidf.setSetPoint(targetAngle);
        //lastTime = getRuntime();
        telemetry.addLine("Testing");
        telemetry.update();
    }

    @Override
    public void loop() {
        pidf.setSetPoint(targetAngle);

        double currentVoltage = absoluteEncoder.getVoltage();
        double currentAngle = (currentVoltage / 3.3) * 360 / 3 - (encoderOffset);



        pidf.setPIDF(kP, kI, kD, kS);
        double steeringPower = pidf.calculate(currentAngle, targetAngle);
        steeringPower = Range.clip(steeringPower, -1.0, 1.0);


        motor1.setPower(steeringPower);
        motor2.setPower(steeringPower);

        telemetry.addData("target_angle_deg", targetAngle);
        telemetry.addData("current_angle_deg", currentAngle);
        //telemetry.addData("error_deg", Math.toDegrees(errorRadians));
        telemetry.addData("steering_power_output", steeringPower);
        telemetry.update();
    }
}