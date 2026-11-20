
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor 3 Max Power", group = "testing")
public class Motor3Test extends OpMode {

    private DcMotorEx motor3;

    @Override
    public void init() {
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        motor3.setPower(1.0);
    }

    @Override
    public void stop() {
        motor3.setPower(0);
    }
}
