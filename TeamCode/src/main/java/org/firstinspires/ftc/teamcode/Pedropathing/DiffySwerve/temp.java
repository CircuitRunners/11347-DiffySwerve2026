//
//package org.firstinspires.ftc.teamcode.Pedropathing.DiffySwerve;
//
//import com.pedropathing.math.Vector2D;
//import com.pedropathing.revhub.drivetrains.SwervePod;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.Range;
//
//import java.util.HashMap;
//import java.util.Map;
//
//public class DiffyPod implements SwervePod {
//
//    private static final double TWO_PI = 2.0 * Math.PI;
//    private static final double DEADBAND = 2.0;
//    private static final double DRIVE_RAMP_RATE = 0.9;
//
//    private final String name;
//    private final DcMotorEx motor1;
//    private final DcMotorEx motor2;
//    private final AnalogInput encoder;
//    private final DiffyPodConfig config;
//
//    // PID state
//    private double integralSum = 0;
//    private double lastError = 0;
//    private double lastTime;
//
//    // Drive ramp state
//    private double currentDrivePower = 0;
//    private double lastDriveTime;
//
//    // Debug
//    private double dbgError = 0;
//    private double dbgSteer = 0;
//    private double dbgDrive = 0;
//
//    public DiffyPod(HardwareMap hardwareMap, DiffyPodConfig config) {
//
//        this.config = config;
//        this.name = config.name.get();
//
//        motor1 = hardwareMap.get(
//                DcMotorEx.class, config.motor1Name.get()
//        );
//
//        motor2 = hardwareMap.get(
//                DcMotorEx.class, config.motor2Name.get()
//        );
//
//        encoder = hardwareMap.get(
//                AnalogInput.class, config.podEncoderName.get()
//        );
//
//        motor1.setDirection(config.motor1Direction.get());
//        motor2.setDirection(config.motor2Direction.get());
//
//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motor1.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                config.motor1Pidf.get()
//        );
//
//        motor2.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                config.motor2Pidf.get()
//        );
//
//        lastTime = getTime();
//        lastDriveTime = getTime();
//
//        stop();
//    }
//
//    @Override
//    public Vector2D getOffset() {
//        return config.podOffset.get();
//    }
//
//    @Override
//    public double getAngle() {
//        return getOffsetAngleRad();
//    }
//
//    @Override
//    public double adjustThetaForEncoder(double wheelTheta) {
//        double angle = config.encoderReversed.get()
//                ? wheelTheta
//                : TWO_PI - wheelTheta;
//
//        return normalize(angle + config.frameOffsetRad.get());
//    }
//
//    @Override
//    public void move(
//            double targetAngleRad,
//            double drivePower,
//            boolean ignoreAngleChanges
//    ) {
//
//        drivePower = Range.clip(drivePower, -1.0, 1.0);
//
//        // Stop when there is no requested drive
//        if (Math.abs(drivePower) < 0.001) {
//            currentDrivePower = 0;
//            motor1.setVelocity(0);
//            motor2.setVelocity(0);
//            return;
//        }
//
//        double targetAngle = Math.toDegrees(
//                adjustThetaForEncoder(targetAngleRad)
//        );
//
//        double currentAngle = Math.toDegrees(getOffsetAngleRad());
//
//        // Calculate shortest steering error
//        double error = wrapDegrees(targetAngle - currentAngle);
//
//        // Optimize steering to avoid turns greater than 90 degrees
//        double driveDirection = 1.0;
//
//        if (error > 90) {
//            error -= 180;
//            driveDirection = -1.0;
//        } else if (error < -90) {
//            error += 180;
//            driveDirection = -1.0;
//        }
//
//        double optimizedError = error;
//
//        // PID timing
//        double now = getTime();
//        double dt = now - lastTime;
//
//        if (dt <= 0 || dt > 0.1) {
//            dt = 0.01;
//        }
//
//        double kP = config.kP.get();
//        double kI = config.kI.get();
//        double kD = config.kD.get();
//        double kS = config.kS.get();
//
//        // Integral
//        integralSum += optimizedError * dt;
//
//        if (kI != 0) {
//            integralSum = Range.clip(
//                    integralSum,
//                    -0.2 / kI,
//                    0.2 / kI
//            );
//        }
//
//        // Derivative
//        double derivative = (optimizedError - lastError) / dt;
//
//        // Static feedforward
//        double feedforward = Math.abs(optimizedError) > DEADBAND
//                ? Math.signum(optimizedError) * kS
//                : 0;
//
//        // Steering output
//        double steeringVelocity =
//                optimizedError * kP
//                        + integralSum * kI
//                        + derivative * kD
//                        + feedforward;
//
//        steeringVelocity = Range.clip(
//                steeringVelocity,
//                -config.maxSteerVelocity.get(),
//                config.maxSteerVelocity.get()
//        );
//
//        lastError = optimizedError;
//        lastTime = now;
//
//        // Drive ramping
//        double driveDt = now - lastDriveTime;
//
//        if (driveDt <= 0) {
//            driveDt = 0.01;
//        }
//
//        double maxDriveChange = DRIVE_RAMP_RATE * driveDt;
//
//        currentDrivePower += Range.clip(
//                drivePower - currentDrivePower,
//                -maxDriveChange,
//                maxDriveChange
//        );
//
//        lastDriveTime = now;
//
//        // Drive velocity
//        double maxMotorVelocity = config.maxMotorVelocity.get();
//
//        double driveVelocity =
//                currentDrivePower * driveDirection * maxMotorVelocity;
//
//        // ignoreAngleChanges bypasses steering and uses direct drive
//        if (ignoreAngleChanges) {
//            steeringVelocity = 0;
//        }
//
//        // Differential motor mixing
//        double ratio = config.motor1Ratio.get();
//
//        double targetVel1 =
//                ratio * (steeringVelocity - driveVelocity);
//
//        double targetVel2 =
//                steeringVelocity + driveVelocity;
//
//        // Normalize both motors together
//        double max = Math.max(
//                maxMotorVelocity,
//                Math.max(
//                        Math.abs(targetVel1),
//                        Math.abs(targetVel2)
//                )
//        );
//
//        motor1.setVelocity(targetVel1 / max * maxMotorVelocity);
//        motor2.setVelocity(targetVel2 / max * maxMotorVelocity);
//
//        // Debug
//        dbgError = optimizedError;
//        dbgSteer = steeringVelocity;
//        dbgDrive = currentDrivePower;
//    }
//
//    public double getRawAngleRad() {
//        double min = config.analogMinVoltage.get();
//        double max = config.analogMaxVoltage.get();
//
//        if (max == min) {
//            return 0;
//        }
//
//        double normalized = Range.clip(
//                (encoder.getVoltage() - min) / (max - min),
//                0,
//                1
//        );
//
//        return normalized * TWO_PI;
//    }
//
//    public double getOffsetAngleRad() {
//        return normalize(
//                getRawAngleRad() - config.angleOffsetRad.get()
//        );
//    }
//
//    public double getSteeringError(double targetAngleRad) {
//        return Math.toRadians(
//                wrapDegrees(
//                        Math.toDegrees(targetAngleRad)
//                                - Math.toDegrees(getOffsetAngleRad())
//                )
//        );
//    }
//
//    public void stop() {
//        currentDrivePower = 0;
//        integralSum = 0;
//        lastError = 0;
//
//        motor1.setVelocity(0);
//        motor2.setVelocity(0);
//    }
//
//    @Override
//    public void setToFloat() {
//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//    }
//
//    @Override
//    public void setToBreak() {
//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public String name() {
//        return name;
//    }
//
//    @Override
//    public Map<String, Object> debug() {
//        Map<String, Object> map = new HashMap<>();
//
//        map.put("name", name);
//        map.put("rawAngleDeg", Math.toDegrees(getRawAngleRad()));
//        map.put("angleAfterOffsetDeg", Math.toDegrees(getOffsetAngleRad()));
//        map.put("errorDeg", dbgError);
//        map.put("steerCmd", dbgSteer);
//        map.put("driveCmd", dbgDrive);
//        map.put("motor1Vel", motor1.getVelocity());
//        map.put("motor2Vel", motor2.getVelocity());
//
//        return map;
//    }
//
//    private double getTime() {
//        return System.nanoTime() / 1e9;
//    }
//
//    private static double wrapDegrees(double angle) {
//        angle %= 360;
//
//        if (angle > 180) angle -= 360;
//        if (angle < -180) angle += 360;
//
//        return angle;
//    }
//
//    private static double normalize(double angle) {
//        double result = angle % TWO_PI;
//        return result < 0 ? result + TWO_PI : result;
//    }
//}
