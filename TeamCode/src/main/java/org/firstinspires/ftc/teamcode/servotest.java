package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="servotest", group="LinearOpmode")
public class servotest extends LinearOpMode {

    boolean manual = true;

    private Follower follower;

    double targetAbsoluteHeading = 0.0;

    @Override
    public void runOpMode() {



        Servo pointer = hardwareMap.get(Servo.class, "pointer");
        double target = 0.5;

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        follower = Constants.create(hardwareMap);

        double desiredServoAngle = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {

                manual = !manual;

            }


            double robotHeading = Math.toDegrees(follower.pose().heading());
            if (manual) {
                targetAbsoluteHeading = follower.pose().heading();
                if (gamepad1.left_bumper) {
                    target += 0.005;
                } else if (gamepad1.right_bumper) {
                    target -= 0.005;
                }

                if (target > 1.0) {
                    target = 1.0;
                }
                if (target < 0.0) {
                    target = 0.0;
                }

                pointer.setPosition(target);
            }
            else {
                desiredServoAngle = targetAbsoluteHeading + robotHeading;

                // Convert angle back to a 0.0 - 1.0 servo position
                target = desiredServoAngle / 180.0;

                while (desiredServoAngle > 180.0)  desiredServoAngle -= 360.0;
                while (desiredServoAngle < -180.0) desiredServoAngle += 360.0;

                // Clamp target between 0.0 and 1.0
                if (target > 1.0) { target = 1.0; }
                if (target < 0.0) { target = 0.0; }

                pointer.setPosition(target);
            }

            double deadzoneStick = 0.05;
            double y = Math.abs(gamepad1.left_stick_y) > deadzoneStick ? -gamepad1.left_stick_y : 0.0;
            double x = Math.abs(gamepad1.left_stick_x) > deadzoneStick ? gamepad1.left_stick_x * 1.1 : 0.0;
            double rx = Math.abs(gamepad1.right_stick_x) > deadzoneStick ? gamepad1.right_stick_x : 0.0;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);



            telemetry.addData("manual?", manual);
            telemetry.addData("Target Position", target);
            telemetry.addData("desired", desiredServoAngle);
            telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
            follower.update();

            sleep(20);
        }
    }
}