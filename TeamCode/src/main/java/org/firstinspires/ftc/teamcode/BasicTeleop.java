package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="teleop", group="LinearOpmode")
public class BasicTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
       // DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            //if (gamepad1.a) {intake.setPower(1);}
            //else {intake.setPower(0);}


            telemetry.addData("Status", "Running");
            telemetry.addData("Front Motors", "Left: %.2f, Right: %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back Motors", "Left: %.2f, Right: %.2f", backLeftPower, backRightPower);
            telemetry.addData("Intake Motor", gamepad1.a);
            telemetry.update();
        }
    }
}
