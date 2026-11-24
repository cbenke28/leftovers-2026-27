package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "rotator", group = "LinearOpMode")
public class rotator extends LinearOpMode {

    private Servo rotator;


    private double servoPosition = 0.5;


    private static final double MAX_STEP_RATE = 0.005;

    @Override
    public void runOpMode() {

        rotator = hardwareMap.get(Servo.class, "rotator");


        rotator.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized - Rotator Ready");
        telemetry.addData("Initial Servo Position", servoPosition);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read trigger analog inputs (checks gamepad1 with gamepad2 fallback)
            double rightTrigger = gamepad1.right_trigger > 0.05 ? gamepad1.right_trigger : gamepad2.right_trigger;
            double leftTrigger = gamepad1.left_trigger > 0.05 ? gamepad1.left_trigger : gamepad2.left_trigger;

            String servoMovement = "STOPPED";
            String shooterMovement = "STOPPED";

            // Trigger controls
            if (rightTrigger > 0.05) {

                servoPosition += MAX_STEP_RATE * rightTrigger;
                servoMovement = "Turning RIGHT (CW)";
                shooterMovement = "Turning LEFT (CCW)";
            } else if (leftTrigger > 0.05) {

                servoPosition -= MAX_STEP_RATE * leftTrigger;
                servoMovement = "Turning LEFT (CCW)";
                shooterMovement = "Turning RIGHT (CW)";
            }

            // Keep servo position bounded between 0.0 and 1.0
            servoPosition = Range.clip(servoPosition, 0.0, 1.0);

            // Update servo hardware position
            rotator.setPosition(servoPosition);

            // Calculate opposite gear (shooter horizontal angle) position
            double shooterGearPosition = 1.0 - servoPosition;

            // Telemetry diagnostics
            telemetry.addData("Status", "Running");
            telemetry.addData("Servo Position", "%.3f", servoPosition);
            telemetry.addData("Shooter Gear Position (Opposite)", "%.3f", shooterGearPosition);
            telemetry.addData("Servo Movement", servoMovement);
            telemetry.addData("Shooter Angle Movement", shooterMovement);
            telemetry.addData("Right Trigger", "%.2f", rightTrigger);
            telemetry.addData("Left Trigger", "%.2f", leftTrigger);
            telemetry.update();

            sleep(10);
        }
    }
}
