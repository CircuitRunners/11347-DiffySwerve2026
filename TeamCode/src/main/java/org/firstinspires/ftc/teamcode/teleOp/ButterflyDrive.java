

package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
@Configurable
@TeleOp(name = "Butterfly Drive")
public class ButterflyDrive extends OpMode {
    private MecanumDrive drive;
    private boolean auto = true;

    public static double butterflyUP = 0.3;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void init(){
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        drive.setServoPositions(0);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.addLine("Initialized");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {

        double forward = -1*gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;

        drive.drive(forward, strafe, rotate);

        if (auto) {
            if (Math.abs(forward) > 0.9 && strafe == 0 && rotate == 0) {
                drive.setServoPositions(0);
            }
            else {
                drive.setServoPositions(butterflyUP);
            }
        }
        else {
            if (gamepad1.dpadUpWasPressed()) {
                drive.setServoPositions(butterflyUP);
            }

            if (gamepad1.dpadDownWasPressed()) {
                drive.setServoPositions(0);
            }
        }

        if (gamepad1.x) { //SQUARE
            if (auto) {
                auto = false;
            }
            else {
                auto = true;
            }
        }

        telemetryM.addData("Mode", auto ? "Automatic" : "Manual");
        telemetryM.addData("Left Servo Position", drive.getLeftServoPosition());
        telemetryM.addData("Right Servo Position", drive.getRightServoPosition());
        telemetryM.update(telemetry);
    }
}