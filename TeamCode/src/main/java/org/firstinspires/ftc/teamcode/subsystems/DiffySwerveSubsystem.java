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
    public static double rotationScale = 0.35;
    public static double encoderAOffset = 75;
    public static double encoderBOffset = 143;
    public static double podDistance = 7.008;
    public static double podARatio = 1;
    public static double podBRatio = 1;
    public static double aSkP = 4.0;
    public static double aSkI = 0.0;
    public static double aSkD = 0.00001;
    public static double aSkS1 = 40.0;
    public static double aSkS2 = 25.0;
    public static double aSkS3 = 10.0;
    public static double bSkP = 2.5;
    public static double bSkI = 0.0;
    public static double bSkD = 0.00001;
    public static double bSkS1 = 40.0;
    public static double bSkS2 = 25.0;
    public static double bSkS3 = 10.0;
//    public static double MAX_LINEAR_SPEED = 1.0;   // inches/sec
//    public static double MAX_ANGULAR_SPEED = 1.0;  // radians/sec
    private double leftAngle;
    private double rightAngle;
    private double finalLeftSpeed;
    private double finalRightSpeed;

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
                aSkS1,
                aSkS2,
                aSkS3,
                15,
                1.0,
                0.0,
                13.60,
                15,
                1.5,
                0.0,
                13.6,
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
                bSkS1,
                bSkS2,
                bSkS3,
                16.0,
                1.0,
                0.0,
                13.5,
                15.0,
                1.0,
                0.0,
                13.5,
                podBRatio

        );


    }

    //Really sigma formula:
    public void drive(double x, double y, double rotation) {
        double podDistFromCent = podDistance / 2;
        rotation = rotation * rotationScale;
        double leftVx = x;
        double leftVy = y - rotation * podDistFromCent;

        double rightVx = x;
        double rightVy = y + rotation * podDistFromCent;

        double leftSpeed = Math.hypot(leftVx, leftVy);
        double rightSpeed = Math.hypot(rightVx, rightVy);

        if (leftSpeed < 0.05 && rightSpeed < 0.05) {
            moduleA.setTarget(leftAngle, 0);
            moduleB.setTarget(rightAngle, 0);
            return;
        }

        leftAngle = Math.toDegrees(Math.atan2(leftVx, leftVy));
        rightAngle = Math.toDegrees(Math.atan2(rightVx, rightVy));

        double maxSpeed = Math.max(leftSpeed, rightSpeed);

        if (maxSpeed > 1.0) {
            leftSpeed /= maxSpeed;
            rightSpeed /= maxSpeed;
        }

        finalLeftSpeed = leftSpeed;
        finalRightSpeed = rightSpeed;

        moduleA.setTarget(leftAngle, finalLeftSpeed);
        moduleB.setTarget(rightAngle, finalRightSpeed);
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

    public double getLeftDriveVel () {
        return finalLeftSpeed;
    }

    public double getRightDriveVel () {
        return finalRightSpeed;
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
