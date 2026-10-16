package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "shooterTwo", group = "LinearOpmode")
public class shooterTwo extends LinearOpMode {

    private DcMotor shooter1;
    private DcMotor shooter2;

    double power;

    @Override
    public void runOpMode() {

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        power = 0.0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {power += 0.1;}
            if (gamepad1.left_bumper) {power -= 0.1;}
            if (power < 0.0) {power = 0.0;}
            if (power > 1.0) {power = 1.0;}

            shooter1.setPower(power);
            shooter2.setPower(power);

            telemetry.addData("power: ", power);
            telemetry.update();

        }
    }
}
