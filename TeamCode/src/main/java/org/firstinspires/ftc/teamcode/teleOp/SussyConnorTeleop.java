

package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Simple 2-Pod Diffy", group = "Drive")
public class SussyConnorTeleop extends LinearOpMode {

    // ====== CONFIG: set your motor names from Robot Config ======
    private DcMotor leftPodA;   // e.g. "leftA"
    private DcMotor leftPodB;   // e.g. "leftB"
    private DcMotor rightPodA;  // e.g. "rightA"
    private DcMotor rightPodB;  // e.g. "rightB"

    // Optional alliance flip
    private boolean isRedAlliance = true;

    private static final double DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        // Hardware map
        leftPodA = hardwareMap.dcMotor.get("leftA");
        leftPodB = hardwareMap.dcMotor.get("leftB");
        rightPodA = hardwareMap.dcMotor.get("rightA");
        rightPodB = hardwareMap.dcMotor.get("rightB");

        // Set directions so +power makes physical sense on your robot
        // You WILL likely need to flip some of these:
        leftPodA.setDirection(DcMotor.Direction.FORWARD);
        leftPodB.setDirection(DcMotor.Direction.FORWARD);
        rightPodA.setDirection(DcMotor.Direction.FORWARD);
        rightPodB.setDirection(DcMotor.Direction.FORWARD);

        // Better drive feel
        leftPodA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPodB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPodA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPodB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready. A=Red alliance, B=Blue alliance");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Tap A/B to change alliance flip
            if (gamepad1.a) isRedAlliance = true;
            if (gamepad1.b) isRedAlliance = false;

            // FTC stick conventions: left_stick_y is negative when pushed forward
            double forward = deadband(-gamepad1.left_stick_y);
            double strafe  = deadband(gamepad1.left_stick_x);
            double rotate  = deadband(gamepad1.right_stick_x);

            // Optional alliance flip
            if (!isRedAlliance) {
                forward = -forward;
                strafe  = -strafe;
            }

            // 2-pod open-loop translation approximation
            double leftTrans  = forward + strafe;
            double rightTrans = forward - strafe;

            // Normalize translation pair
            double transScale = Math.max(1.0, Math.max(Math.abs(leftTrans), Math.abs(rightTrans)));
            leftTrans  /= transScale;
            rightTrans /= transScale;

            // Diffy mix per pod (your requested logic)
            double leftA  = rotate + leftTrans;
            double leftB  = rotate - leftTrans;
            double rightA = rotate + rightTrans;
            double rightB = rotate - rightTrans;

            // Final normalize all 4 motors together
            double max = Math.max(
                    Math.max(Math.abs(leftA), Math.abs(leftB)),
                    Math.max(Math.abs(rightA), Math.abs(rightB))
            );
            if (max < 1.0) max = 1.0;

            leftA  = Range.clip(leftA / max, -1.0, 1.0);
            leftB  = Range.clip(leftB / max, -1.0, 1.0);
            rightA = Range.clip(rightA / max, -1.0, 1.0);
            rightB = Range.clip(rightB / max, -1.0, 1.0);

            // Apply powers
            leftPodA.setPower(leftA);
            leftPodB.setPower(leftB);
            rightPodA.setPower(rightA);
            rightPodB.setPower(rightB);

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("fwd/str/rot", "%.2f  %.2f  %.2f", forward, strafe, rotate);
            telemetry.addData("L(A,B)", "%.2f  %.2f", leftA, leftB);
            telemetry.addData("R(A,B)", "%.2f  %.2f", rightA, rightB);
            telemetry.update();
        }
    }

    private double deadband(double x) {
        return Math.abs(x) < DEADBAND ? 0.0 : x;
    }
}
