package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
public class SwerveDrive {
    //left pods
    public DcMotorEx leftMotor1;
    public DcMotorEx leftMotor2;
    public AnalogInput leftAbsoluteEncoder;
    private SwervePod leftPod;
    //right pods
    public DcMotorEx rightMotor1;
    public DcMotorEx rightMotor2;
    public AnalogInput rightAbsoluteEncoder;
    private SwervePod rightPod;

    private double radius;

    private double LEFT_ENCODER_OFFSET;
    private double RIGHT_ENCODER_OFFSET;
    public void init(HardwareMap hardwareMap) {
        leftMotor1 = hardwareMap.get(DcMotorEx.class, "leftMotor1");
        leftMotor2 = hardwareMap.get(DcMotorEx.class, "leftMotor2");
        rightMotor1 = hardwareMap.get(DcMotorEx.class, "rightMotor1");
        rightMotor2 = hardwareMap.get(DcMotorEx.class, "rightMotor2");

        leftAbsoluteEncoder = hardwareMap.get(AnalogInput.class, "leftEncoder");
        rightAbsoluteEncoder = hardwareMap.get(AnalogInput.class, "rightEncoder");

        leftPod = new SwervePod(leftMotor1, leftMotor2, leftAbsoluteEncoder, LEFT_ENCODER_OFFSET);
        rightPod = new SwervePod(rightMotor1, rightMotor2, rightAbsoluteEncoder, RIGHT_ENCODER_OFFSET);

        leftPod.setPID(0.004, 0.0, 0.0001, 0.08);
        rightPod.setPID(0.004, 0.0, 0.0001, 0.08);

        leftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getLeftAngle (){
        return leftPod.getModuleAngle();
    }

    public double getRightAngle() {
        return rightPod.getModuleAngle();
    }

    public void drive(double forward, double strafe, double rotate, double currentTime) {
        double rotateLeft = rotate * radius;
        double rotateRight = -rotate * radius;

        double leftPodX = strafe;
        double leftPodY = forward + rotateLeft;
        double rightPodX = strafe;
        double rightPodY = forward + rotateRight;

        double leftTargetAngle = Math.toDegrees(Math.atan2(leftPodX, leftPodY));
        double leftTargetPower = Math.hypot(leftPodX, leftPodY);

        double rightTargetAngle = Math.toDegrees(Math.atan2(rightPodX, rightPodY));
        double rightTargetPower = Math.hypot(rightPodX, rightPodY);

        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(leftTargetPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(rightTargetPower));
        leftTargetPower /= maxSpeed;
        rightTargetPower /= maxSpeed;

        leftPod.update(leftTargetAngle, leftTargetPower, currentTime);
        rightPod.update(rightTargetAngle, rightTargetPower, currentTime);
    }
}
