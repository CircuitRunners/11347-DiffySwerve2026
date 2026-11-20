package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
@TeleOp(name = "PID testing COMBINED", group = "testing")
public class combinedMotorPID extends OpMode{
    private DcMotorEx mA1;
    private DcMotorEx mA2;
    public static double mA1P = 1.7;
    public static double mA1I = 0.0;
    public static double mA1D = 0.0;
    public static double mA1F = 12.7;
    public static double mA2P = 15.0;
    public static double mA2I = 0.0;
    public static double mA2D = 1.0;
    public static double mA2F = 13.8;
    public static double targetAV1 = 0.0;
    public static double targetAV2 = 0.0;

    //------------------------------
    private DcMotorEx mB1;
    private DcMotorEx mB2;
    public static double mB1P = 15.0;
    public static double mB1I = 0.0;
    public static double mB1D = 0.0;
    public static double mB1F = 13.0;
    public static double mB2P = 15.0;
    public static double mB2I = 0.0;
    public static double mB2D = 1.0;
    public static double mB2F = 13.0;
    public static double targetBV1 = 0.0;
    public static double targetBV2 = 0.0;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void init() {
        mA1 = hardwareMap.get(DcMotorEx.class, "motor1");
        mA2 = hardwareMap.get(DcMotorEx.class, "motor2");

        mA1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mA2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mA1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mA2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mB1 = hardwareMap.get(DcMotorEx.class, "motor3");
        mB2 = hardwareMap.get(DcMotorEx.class, "motor4");

        mB1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mB2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Required for Control Hub hardware-level velocity PIDF control
        mB1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mB2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.addLine("Initialized");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        mA1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(mA1P, mA1I, mA1D, mA1F));
        mA2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(mA2P, mA2I, mA2D, mA2F));

        mA1.setVelocity(targetAV1);
        mA2.setVelocity(targetAV2);

        mB1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(mB1P, mB1I, mB1D, mB1F));
        mB2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(mB2P, mB2I, mB2D, mB2F));

        mB1.setVelocity(targetBV1);
        mB2.setVelocity(targetBV2);

        double currentBVel1 = mB1.getVelocity();
        double currentBVel2 = mB2.getVelocity();

        double currentAVel1 = mA1.getVelocity();
        double currentAVel2 = mA2.getVelocity();

        telemetryM.addData("Motor A1 Target Vel", targetAV1);
        telemetryM.addData("Motor A1 Measured Vel", currentAVel1);

        telemetryM.addData("Motor A2 Target Vel", targetAV2);
        telemetryM.addData("Motor A2 Measured Vel", currentAVel2);

        telemetryM.addData("Motor B1 Target Vel", targetBV1);
        telemetryM.addData("Motor B1 Measured Vel", currentBVel1);

        telemetryM.addData("Motor 2 Target Vel", targetBV2);
        telemetryM.addData("Motor 2 Measured Vel", currentBVel2);

        telemetryM.update(telemetry);
    }
}

