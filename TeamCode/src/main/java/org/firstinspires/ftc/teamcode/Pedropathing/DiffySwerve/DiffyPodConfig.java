//package org.firstinspires.ftc.teamcode.Pedropathing.DiffySwerve;
//
//import static com.pedropathing.config.Validator.nonnegative;
//
//import com.pedropathing.config.ConfigVar;
//import com.pedropathing.config.Configuration;
//import com.pedropathing.math.Vector2D;
//import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//public class DiffyPodConfig {
//    public final ConfigVar<String> name = ConfigVar.required();
//    public final ConfigVar<String> motor1Name = ConfigVar.required();
//    public final ConfigVar<String> motor2Name = ConfigVar.required();
//    public final ConfigVar<String> podEncoderName = ConfigVar.required();
//
//    public final ConfigVar<DcMotorEx.Direction> motor1Direction = ConfigVar.required();
//    public final ConfigVar<DcMotorEx.Direction> motor2Direction = ConfigVar.required();
//    public final ConfigVar<PIDFCoefficients> motor1Pidf = ConfigVar.required();
//    public final ConfigVar<PIDFCoefficients> motor2Pidf = ConfigVar.required();
//    public final ConfigVar<Double> maxMotorVelocity = ConfigVar.of(1800.0, nonnegative());
//    public final ConfigVar<Double> maxSteerVelocity = ConfigVar.of(500.0, nonnegative());
//    public final ConfigVar<Double> angleOffsetRad = ConfigVar.of(0.0);
//
//    public final ConfigVar<Vector2D> podOffset = ConfigVar.required();
//
//    public final ConfigVar<Double> analogMinVoltage = ConfigVar.of(0.0, nonnegative());
//    public final ConfigVar<Double> analogMaxVoltage = ConfigVar.of(3.3, nonnegative());
//    public final ConfigVar<Boolean> encoderReversed = ConfigVar.of(false);
//    public final ConfigVar<Double> velocityCachingThreshold = ConfigVar.of(5.0, nonnegative());
//    public DiffyPodConfig(Configuration<DiffyPodConfig> config) {
//        config.configure(this);
//    }
//}
