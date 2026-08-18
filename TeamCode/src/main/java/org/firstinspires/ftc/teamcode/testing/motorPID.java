package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
@TeleOp(name = "PID testing for motors", group = "testing")
public class motorPID extends OpMode{
    private DcMotorEx m1;
    private DcMotorEx m2;
    public static double m1P = 0.0;
    public static double m1I = 0.0;
    public static double m1D = 0.0;
    public static double m1F = 0.0;
    public static double m2P = 0.0;
    public static double m2I = 0.0;
    public static double m2D = 0.0;
    public static double m2F = 0.0;
    public static double targetV1 = 0.0;
    public static double targetV2 = 0.0;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotorEx.class, "motor1");
        m2 = hardwareMap.get(DcMotorEx.class, "motor2");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Required for Control Hub hardware-level velocity PIDF control
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.addLine("Initialized");
        telemetryM.update(telemetry);
    }
//max speed 2000
    @Override
    public void loop() {
        m1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m1P, m1I, m1D, m1F));
        m2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m2P, m2I, m2D, m2F));

        m1.setVelocity(targetV1);
        m2.setVelocity(targetV2);

        double currentVel1 = m1.getVelocity();
        double currentVel2 = m2.getVelocity();

        telemetryM.addData("Motor 1 Target Vel", targetV1);
        telemetryM.addData("Motor 1 Measured Vel", currentVel1);
        telemetryM.addData("Motor 1 Error", targetV1 - currentVel1);

        telemetryM.addData("Motor 2 Target Vel", targetV2);
        telemetryM.addData("Motor 2 Measured Vel", currentVel2);
        telemetryM.addData("Motor 2 Error", targetV2 - currentVel2);

        telemetryM.update(telemetry);
    }
}
