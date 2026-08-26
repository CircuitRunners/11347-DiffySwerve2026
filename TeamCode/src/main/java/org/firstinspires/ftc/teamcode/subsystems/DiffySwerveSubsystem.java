package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.SwervePod;
@Configurable
public class DiffySwerveSubsystem {
    private final SwervePod moduleA;
    private final SwervePod moduleB;
    public static double encoderAOffset = 50;
    public static double encoderBOffset = -50;
    public static double podDistance = 10.728;
    public static double podARatio = -1;
    public static double podBRatio = 1;
    public static double aSkP = 5.85;
    public static double aSkI = 0.0;
    public static double aSkD = 0.038;
    public static double aSkS = 80;
    public static double bSkP = 3.0;
    public static double bSkI = 0.0;
    public static double bSkD = 0.026;
    public static double bSkS = 80;
    private double leftAngle;
    private double rightAngle;


    //POD A PID

    public DiffySwerveSubsystem (HardwareMap hardwareMap) {
        moduleA = new SwervePod(
                hardwareMap.get(DcMotorEx.class, "motor1"),
                hardwareMap.get(DcMotorEx.class, "motor2"),
                hardwareMap.get(AnalogInput.class, "encoderA"),
                encoderAOffset,
                aSkP,
                aSkI,
                aSkD,
                aSkS, //220
                4.3,
                0.0,
                0.0,
                6.0,
                15.0,
                0.0,
                1.0,
                13.0,
                podARatio
        );
        moduleB = new SwervePod(
                hardwareMap.get(DcMotorEx.class, "motor3"),
                hardwareMap.get(DcMotorEx.class, "motor4"),
                hardwareMap.get(AnalogInput.class, "encoderB"),
                encoderBOffset,
                bSkP,
                bSkI,
                bSkD,
                bSkS, //150
                15.0,
                0.0,
                0.0,
                13.0,
                15.0,
                0.0,
                1.0,
                13.0,
                podBRatio

        );


    }

    //Really sigma formula:
    public void drive(double x, double y, double rotation) {
        double podDistFromCent = podDistance / 2;
        double leftVx = x;
        double leftVy = y - rotation * podDistFromCent;

        double rightVx = x;
        double rightVy = y + rotation * podDistFromCent;

        double leftSpeed = Math.hypot(leftVx, leftVy);
        double rightSpeed = Math.hypot(rightVx, rightVy);

        leftAngle = Math.toDegrees(Math.atan2(leftVx, leftVy));
        rightAngle = Math.toDegrees(Math.atan2(rightVx, rightVy));

        double maxSpeed = Math.max(leftSpeed, rightSpeed);

        if (maxSpeed > 1.0) {
            leftSpeed /= maxSpeed;
            rightSpeed /= maxSpeed;
        }

        moduleA.setTarget(leftAngle, leftSpeed);
        moduleB.setTarget(rightAngle, rightSpeed);
    }

    public double getLeftAngle () {
        return leftAngle;
    }

    public double getRightAngle () {
        return rightAngle;
    }

    public double getLeftError () {
        return moduleA.getSteeringError(leftAngle);
    }

    public double getRightError () {
        return moduleB.getSteeringError(rightAngle);
    }

    public void update() {
        moduleA.update();
        moduleB.update();
    }

    public void stop () {
        moduleA.stop();
        moduleB.stop();
    }
}
