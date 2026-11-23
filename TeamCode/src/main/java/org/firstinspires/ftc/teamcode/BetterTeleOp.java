package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TransferQueue;

@TeleOp(name = "BetterTeleOp")
public class BetterTeleOp extends OpMode {

    private Follower follower;

    private double targetHeading = 0.0;
    private boolean holdingHeading = false;
    private boolean intakeOn = false;
    private boolean transferOn = false;
    private double shooterpower = 0.0;

    Servo pointer;

    int target = 0;
    double servot;

    private static final double correctionPower = 2.0;
    private static final double allowable = 0.02;
    private static final double deadzoneTurn = 0.05;

    private static final double hiveX = 64.3;
    private static final double hiveY = 58.8;

    DcMotor intake;
    DcMotor transfer;
    DcMotor shooter1;
    DcMotor shooter2;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        pointer = hardwareMap.get(Servo.class, "shootangle");


    }

    @Override
    public void start() {
        targetHeading = follower.pose().heading();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double turnPower;
        double deadzoneStick = 0.05;
        double forward = Math.abs(gamepad1.left_stick_y) > deadzoneStick ? -gamepad1.left_stick_y : 0.0;
        double strafe = Math.abs(gamepad1.left_stick_x) > deadzoneStick ? -gamepad1.left_stick_x : 0.0;
        double turnInput = -gamepad1.right_stick_x;
        boolean facingTarget = gamepad1.b;

        if (gamepad1.a) {
            follower.setX(0);
            follower.setY(0);
            follower.setHeading(0);
            targetHeading = 0.0;
        }

        if (gamepad1.xWasPressed()) {intakeOn = !intakeOn;}
        if (gamepad1.yWasPressed()) {transferOn = !transferOn;}


        double currentX = follower.pose().x();
        double currentY = follower.pose().y();
        double currentHeading = follower.pose().heading();

        if (facingTarget) {
            double dx = hiveX - currentX;
            double dy = hiveY - currentY;

            targetHeading = Math.atan2(dy, dx);

            double error = normalizeAngle(targetHeading - currentHeading);

            if (Math.abs(error) < allowable) {
                turnPower = 0.0;
            } else {
                turnPower = error * correctionPower;
                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
            }
            holdingHeading = false;
        }
        else if (Math.abs(turnInput) > deadzoneTurn) {
            turnPower = turnInput;
            targetHeading = currentHeading;
            holdingHeading = false;
        }
        else {
            if (!holdingHeading) {
                targetHeading = currentHeading;
                holdingHeading = true;
            }

            double error = normalizeAngle(targetHeading - currentHeading);

            if (Math.abs(error) < allowable) {
                turnPower = 0.0;
            } else {
                turnPower = error * correctionPower;
                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
            }
        }

        //if (intakeOn) {intake.setPower(1);}
        //else {intake.setPower(0);}
        if (transferOn) {transfer.setPower(1);}
        else {transfer.setPower(0);}


        if (gamepad1.leftTriggerWasPressed()) {
            target -= 5;
        } else if (gamepad1.rightTriggerWasPressed()) {
            target += 5;
        }
        if (gamepad1.leftBumperWasPressed()) {
            shooterpower -= 0.05;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterpower += 0.05;
        }
        if (shooterpower > 1.0) {shooterpower = 1.0;}
        if (shooterpower < 0.0) {shooterpower = 0.0;}

        shooter1.setPower(shooterpower);
        shooter2.setPower(shooterpower);

        target = Math.max(0, Math.min(360, target));
        servot = target / 300.0;
        pointer.setPosition(servot);

        follower.manual(forward, strafe, turnPower);

        telemetry.addData("Auto Heading", facingTarget);
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("Shooter Power", shooterpower);
        telemetry.addData("Hood Angle", target * 300);
        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        telemetry.addData("Intake", intakeOn);
        telemetry.addData("Transfer", transferOn);
        follower.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}