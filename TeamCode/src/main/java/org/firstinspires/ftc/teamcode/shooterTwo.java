package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "shooterTwo", group = "LinearOpmode")
public class shooterTwo extends LinearOpMode {

    private DcMotor shooter1;
    private DcMotor shooter2;

    @Override
    public void runOpMode() {

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            shooter1.setPower(gamepad1.right_stick_y);
            shooter2.setPower(gamepad1.right_stick_y);

            telemetry.addData("power: ", gamepad1.right_stick_y);
            telemetry.update();

        }
    }
}
