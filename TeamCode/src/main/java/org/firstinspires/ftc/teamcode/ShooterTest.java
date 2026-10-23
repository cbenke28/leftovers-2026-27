package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ShooterTest", group = "Testing")
public class ShooterTest extends LinearOpMode {

    private DcMotor shooter1;
    private DcMotor shooter2;

    @Override
    public void runOpMode() {
        // Retrieve shooter motors from hardware map
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized - Ready to test shooter");
        telemetry.addData("Presets", "A = 25% | B = 30% | Y = 35% | X = Stop (0%)");
        telemetry.addData("D-Pad", "Up = +1% | Down = -1% (Tap to adjust)");
        telemetry.addData("Trigger", "Right Trigger = Variable Power");
        telemetry.update();

        waitForStart();

        double shooterPower = 0.0;

        // Variables for single-press D-Pad detection
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (opModeIsActive()) {
            // Preset power buttons
            if (gamepad1.a) {
                shooterPower = 0.25;   // 25% power
            } else if (gamepad1.b) {
                shooterPower = 0.30;   // 30% power
            } else if (gamepad1.y) {
                shooterPower = 0.35;   // 35% power
            } else if (gamepad1.x) {
                shooterPower = 0.0;    // Stop motor
            }

            // Right trigger override for analog speed
            if (gamepad1.right_trigger > 0.05) {
                shooterPower = gamepad1.right_trigger;
            }

            // D-Pad fine tuning: adjust by 1% per tap
            if (gamepad1.dpad_up && !lastDpadUp) {
                shooterPower += 0.01;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                shooterPower -= 0.01;
            }

            // Update button tracking states
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Keep power bounded between 0.0 (0%) and 1.0 (100%)
            shooterPower = Range.clip(shooterPower, 0.0, 1.0);

            // Set power to both shooter motors
            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // Display current shooter telemetry
            telemetry.addData("Target Power", "%.2f", shooterPower);
            telemetry.addData("Actual Motor 1 Power", "%.2f", shooter1.getPower());
            telemetry.addData("Actual Motor 2 Power", "%.2f", shooter2.getPower());
            telemetry.update();
        }
    }
}
