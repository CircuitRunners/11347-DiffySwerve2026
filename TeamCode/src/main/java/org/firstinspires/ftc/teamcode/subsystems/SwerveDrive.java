//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
///**
// * TwoWheelDiffySwerve subsystem controls two differential swerve modules (4 motors total).
// * Provides live PIDF coefficient updates, velocity targeting, and dashboard integration.
// */
//@Config
//public class SwerveDrive {
//    // --- Hardware ---
//    private final DcMotorEx m1; // Module 1 Motor 1
//    private final DcMotorEx m2; // Module 1 Motor 2
//    private final DcMotorEx m3; // Module 2 Motor 1
//    private final DcMotorEx m4; // Module 2 Motor 2
//
//    // --- Dashboard & Telemetry ---
//    private final FtcDashboard dash;
//    private final Telemetry telemetry;
//
//    // --- Module 1 PIDF Coefficients ---
//    public static double m1P = 4.3;
//    public static double m1I = 0.0;
//    public static double m1D = 0.0;
//    public static double m1F = 6.0;
//
//    public static double m2P = 15.0;
//    public static double m2I = 0.0;
//    public static double m2D = 1.0;
//    public static double m2F = 13.0;
//
//    // --- Module 2 PIDF Coefficients ---
//    public static double m3P = 4.3;
//    public static double m3I = 0.0;
//    public static double m3D = 0.0;
//    public static double m3F = 6.0;
//
//    public static double m4P = 15.0;
//    public static double m4I = 0.0;
//    public static double m4D = 1.0;
//    public static double m4F = 13.0;
//
//    // --- Target Velocities & Direction Inversions ---
//    private double targetV1 = 0.0;
//    private double targetV2 = 0.0;
//    private double targetV3 = 0.0;
//    private double targetV4 = 0.0;
//
//    public static int invertM1 = -1;
//    public static int invertM2 = 1;
//    public static int invertM3 = -1;
//    public static int invertM4 = 1;
//
//    /**
//     * Initializes the 2-wheel differential swerve subsystem.
//     * @param hardwareMap HardwareMap passed from the OpMode
//     * @param telemetry Telemetry instance for phone/dashboard output
//     */
//    public Init(HardwareMap hardwareMap, Telemetry telemetry) {
//        // Initialize FTC Dashboard Telemetry integration
//        dash = FtcDashboard.getInstance();
//        this.telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
//
//        // Initialize Hardware
//        m1 = hardwareMap.get(DcMotorEx.class, "m1");
//        m2 = hardwareMap.get(DcMotorEx.class, "m2");
//        m3 = hardwareMap.get(DcMotorEx.class, "m3");
//        m4 = hardwareMap.get(DcMotorEx.class, "m4");
//
//        // Configure Zero Power Behaviors
//        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Set Run Modes
//        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Apply initial PIDF values
//        applyPIDF();
//
//        this.telemetry.addLine("Two-Wheel Diffy Swerve Subsystem Initialized");
//    }
//
//    /** Applies the current PIDF coefficients to each motor. */
//    public void applyPIDF() {
//        if (m1 != null) m1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m1P, m1I, m1D, m1F));
//        if (m2 != null) m2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m2P, m2I, m2D, m2F));
//        if (m3 != null) m3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m3P, m3I, m3D, m3F));
//        if (m4 != null) m4.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(m4P, m4I, m4D, m4F));
//    }
//
//    /**
//     * Sets target motor velocities for both modules.
//     * @param v1 Target velocity for Module 1 Motor 1 (ticks/s)
//     * @param v2 Target velocity for Module 1 Motor 2 (ticks/s)
//     * @param v3 Target velocity for Module 2 Motor 1 (ticks/s)
//     * @param v4 Target velocity for Module 2 Motor 2 (ticks/s)
//     */
//    public void setMotorVelocities(double v1, double v2, double v3, double v4) {
//        this.targetV1 = v1;
//        this.targetV2 = v2;
//        this.targetV3 = v3;
//        this.targetV4 = v4;
//    }
//
//    /**
//     * Helper to command differential outputs per module.
//     * Differential swerve speed = (v_a + v_b) / 2
//     * Differential swerve angle velocity = (v_a - v_b) / 2
//     */
//    public void setModuleOutputs(double mod1Speed, double mod1Steer, double mod2Speed, double mod2Steer) {
//        setMotorVelocities(
//                mod1Speed + mod1Steer,
//                mod1Speed - mod1Steer,
//                mod2Speed + mod2Steer,
//                mod2Speed - mod2Steer
//        );
//    }
//
//    /** Updates motor speeds, applies PIDF coefficients, and pushes telemetry. */
//    public void update() {
//        // Re-apply PIDF values in case tuning constants were altered dynamically
//        applyPIDF();
//
//        // Apply commanded velocities with inversion constants
//        m1.setVelocity(invertM1 * targetV1);
//        m2.setVelocity(invertM2 * targetV2);
//        m3.setVelocity(invertM3 * targetV3);
//        m4.setVelocity(invertM4 * targetV4);
//
//        // Read measured velocities
//        double vel1 = m1.getVelocity();
//        double vel2 = m2.getVelocity();
//        double vel3 = m3.getVelocity();
//        double vel4 = m4.getVelocity();
//
//        // Telemetry
//        telemetry.addData("M1 Target", targetV1);
//        telemetry.addData("M1 Measured", vel1);
//        telemetry.addData("M1 Error", targetV1 - vel1);
//
//        telemetry.addData("M2 Target", targetV2);
//        telemetry.addData("M2 Measured", vel2);
//        telemetry.addData("M2 Error", targetV2 - vel2);
//
//        telemetry.addData("M3 Target", targetV3);
//        telemetry.addData("M3 Measured", vel3);
//        telemetry.addData("M3 Error", targetV3 - vel3);
//
//        telemetry.addData("M4 Target", targetV4);
//        telemetry.addData("M4 Measured", vel4);
//        telemetry.addData("M4 Error", targetV4 - vel4);
//
//        telemetry.update();
//    }
//
//    public void periodic() {
//        update();
//    }
//
//    /** Emergency stop to halt all motors instantly. */
//    public void eStop() {
//        setMotorVelocities(0, 0, 0, 0);
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
//    }
//}