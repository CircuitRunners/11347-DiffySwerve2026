package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

public class SwerveTeleOp extends OpMode{
    private SwerveDrive drive;
    private ElapsedTime runtime;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void init(){
        drive = new SwerveDrive();
        drive.init(hardwareMap);

        runtime = new ElapsedTime();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.addLine("Initialized");
        telemetryM.update(telemetry);
    }
    @Override
    public void init_loop() {
        telemetry.addData("Left Pod Absolute Angle", drive.getLeftAngle());
        telemetry.addData("Right Pod Absolute Angle", drive.getRightAngle());
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        double currentTime = runtime.seconds();

        drive.drive(forward, strafe, rotate, currentTime);

        telemetryM.addData("Left Pod Current Angle", drive.getLeftAngle());
        telemetryM.addData("Right Pod Current Angle", drive.getRightAngle());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        drive.leftMotor1.setPower(0);
        drive.leftMotor2.setPower(0);
        drive.rightMotor1.setPower(0);
        drive.rightMotor2.setPower(0);
    }
}
