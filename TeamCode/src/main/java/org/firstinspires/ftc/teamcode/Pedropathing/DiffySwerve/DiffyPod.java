//package org.firstinspires.ftc.teamcode.Pedropathing.DiffySwerve;
//
//import com.pedropathing.math.Vector2D;
//import com.pedropathing.revhub.drivetrains.SwervePod;
//import com.qualcomm.robotcore.hardware.*;
//import com.qualcomm.robotcore.util.Range;
//
//public class DiffyPod implements SwervePod {
//    private final String name;
//    private final AnalogInput podEncoder;
//    private final DcMotorEx motor1;
//    private final DcMotorEx motor2;
//    private final DiffyPodConfig config;
//    private double integralSum = 0;
//    private double lastError = 0;
//    private double lastTime;
//    private double currentDrivePower = 0;
//    private double lastDriveTime = 0;
//
//
//    public DiffyPod(HardwareMap hardwareMap, DiffyPodConfig config) {
//        this.config = config;
//        this.name = config.name.get();
//
//        this.motor1 = hardwareMap.get(DcMotorEx.class, config.motor1Name.get());
//        this.motor2 = hardwareMap.get(DcMotorEx.class, config.motor2Name.get());
//        this.podEncoder = hardwareMap.get(AnalogInput.class, config.podEncoderName.get());
//
//        motor1.setDirection(config.motor1Direction.get());
//        motor2.setDirection(config.motor2Direction.get());
//
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.motor1Pidf.get());
//        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.motor2Pidf.get());
//
//        //setToFloat();
//    }
//
//    @Override
//    public Vector2D getOffset() {
//        return config.podOffset.get();
//    }
//
//    @Override
//    public double getAngle() {
//        return getAngleAfterOffsetRad();
//    }
//
////    @Override
////    public void setToFloat() {
////        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////    }
////
////    @Override
////    public void setToBreak() {
////        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////    }
//
//    @Override
//    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChange) {
//        drivePower = Range.clip(drivePower, -1.0, 1.0);
//        if (Math.abs(drivePower) < 0.001) {
//            currentDrivePower = 0.0;
//
//            motor1.setVelocity(0);
//            motor2.setVelocity(0);
//
//            return;
//        }
//
//        double targetAngle = Math.toDegrees(
//                adjustThetaForEncoder(targetAngleRad)
//        );
//
//        double currentAngle = Math.toDegrees(getOffsetAngleRad());
//    }
//
//    public double getOffsetAngleRad() {
//        return normalize(
//                getRawAngleRad() - config.angleOffsetRad.get()
//        );
//    }
//}
