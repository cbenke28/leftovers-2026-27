package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servotest", group="LinearOpmode")
public class servotest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo pointer = hardwareMap.get(Servo.class, "pointer");
        double target = 0.5;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_stick_x > 0.2) {
                target += 0.005;
            } else if (gamepad1.left_stick_x < -0.2) {
                target -= 0.005;
            }

            if (target > 1.0) { target = 1.0; }
            if (target < 0.0) { target = 0.0; }

            pointer.setPosition(target);

            telemetry.addData("Target Position", target);
            telemetry.update();

            sleep(20);
        }
    }
}
