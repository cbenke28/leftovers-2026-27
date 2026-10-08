package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ShooterTest", group = "Testing")
public class ShooterTest extends LinearOpMode {

    private DcMotor shooter;

    @Override
    public void runOpMode() {
        // Retrieve the single shooter motor from hardware map (configured as "shooter")
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // Optional: set motor direction or zero power behavior if needed
        // shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized - Ready to test shooter");
        telemetry.addData("Controls", "A = 50% | B = 75% | Y = 100% | X = Stop (0%)");
        telemetry.addData("Trigger Control", "Right Trigger = Variable Power");
        telemetry.update();

        waitForStart();

        double shooterPower = 0.0;

        while (opModeIsActive()) {
            // Option 1: Set power using preset buttons
            if (gamepad1.a) {
                shooterPower = 0.5;   // 50% power
            } else if (gamepad1.b) {
                shooterPower = 0.55;  // 75% power
            } else if (gamepad1.y) {
                shooterPower = 0.6;   // 100% power
            } else if (gamepad1.x) {
                shooterPower = 0.0;   // Stop motor
            }
            
            // Option 2: Right trigger override for analog variable speed
            if (gamepad1.right_trigger > 0.05) {
                shooterPower = gamepad1.right_trigger;
            }

            // Set the motor power
            shooter.setPower(shooterPower);

            // Display current shooter telemetry to Driver Station
            telemetry.addData("Target Power", "%.2f", shooterPower);
            telemetry.addData("Actual Motor Power", "%.2f", shooter.getPower());
            telemetry.update();
        }
    }
}
