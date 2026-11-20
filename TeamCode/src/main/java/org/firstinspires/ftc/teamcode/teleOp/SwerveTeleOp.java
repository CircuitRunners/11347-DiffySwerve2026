package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DiffySwerveSubsystem;

@Configurable
@TeleOp(name = "Diffy TeleOp")
public class SwerveTeleOp extends OpMode{
    private DiffySwerveSubsystem swerve;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    @Override
    public void init() {
        swerve = new DiffySwerveSubsystem((hardwareMap));

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //lastTime = getRuntime();
        telemetryM.addLine("Testing");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double y = -(double) Math.round(gamepad1.left_stick_y * 100) / 100;
        double x = (double) Math.round(gamepad1.left_stick_x * 100) / 100;

        double rotation = -gamepad1.right_stick_x;

        swerve.drive(x, y, rotation);

        swerve.update();

        telemetryM.addData("Target A:", swerve.getLeftAngle());
        telemetryM.addData("Target B:", swerve.getRightAngle());
        telemetryM.addData("Error A:", swerve.getLeftError());
        telemetryM.addData("Error B:", swerve.getRightError());
        telemetryM.addData("x:", x);
        telemetryM.addData("y:", y);
        telemetryM.addData("rotation:", rotation);

        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        swerve.stop();
    }
}
