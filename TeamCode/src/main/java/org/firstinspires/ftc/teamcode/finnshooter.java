package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "finn shooter", group = "Testing")
public class finnshooter extends LinearOpMode {

    private DcMotor m1;
    private DcMotor m2;
    private Servo s1;
    private Servo s2;
    private Servo pivot;

    double power;
    double angle;
    double heading;

    private String mode;

    @Override
    public void runOpMode(){

        m1 = hardwareMap.get(DcMotor.class, "shooter1");
        m2 = hardwareMap.get(DcMotor.class, "shooter2");
        s1 = hardwareMap.get(Servo.class, "hood1");
        s2 = hardwareMap.get(Servo.class, "hood2");
        pivot = hardwareMap.get(Servo.class, "rotator");

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.FORWARD);
        s1.setDirection(Servo.Direction.REVERSE);
        s2.setDirection(Servo.Direction.FORWARD);

        power = 0.3;
        angle = 0.0;
        heading = 0.0;

        mode = "motor";

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.aWasPressed()){
                mode = (mode.equals("motor")) ? "angle" : "motor";
            }

            if (mode.equals("motor")){

                if (gamepad1.leftBumperWasPressed()) {
                    power -= 0.01;
                }
                if (gamepad1.rightBumperWasPressed()){
                    power += 0.01;
                }

                power = Range.clip(power, 0.0, 1.0);

            } else if (mode.equals("angle")){

                if (gamepad1.leftBumperWasPressed()) {
                    angle -= 20;
                }
                if (gamepad1.rightBumperWasPressed()){
                    angle += 20;
                }

            }

            m1.setPower(power);
            m2.setPower(power);
            s1.setPosition(servoNormalizer(angle));
            s2.setPosition(servoNormalizer(angle));

            telemetry.addData("Mode", mode);
            telemetry.addData("Power", power);
            telemetry.addData("Angle", angle);
            telemetry.update();
        }
    }

    private double servoNormalizer(double angle) {

        double clippedAngle = Range.clip(angle, 0.0, 300.0);
        return clippedAngle / 300.0;

    }

}