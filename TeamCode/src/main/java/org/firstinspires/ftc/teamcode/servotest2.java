package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;

@TeleOp(name="servotest2", group="LinearOpmode")
public class servotest2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo pointer = hardwareMap.get(Servo.class, "pointer");

        // Force the hub to output goBILDA's full 500-2500 microseconds range
        ((PwmControl) pointer).setPwmRange(new PwmControl.PwmRange(500, 2500));

        int target = 0;
        double servot;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.leftBumperWasPressed()) {
                target -= 5;
            } else if (gamepad1.rightBumperWasPressed()) {
                target += 5;
            }

            // Clamp target between 0.0 and 1.0
            target = Math.max(0, Math.min(360, target));

            servot = target / 1800.0;

            pointer.setPosition(servot);

            telemetry.addData("Target Position", target);
            telemetry.update();
        }
    }
}