package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "BetterTeleOp")
public class BetterTeleOp extends OpMode {

    private Follower follower;
    private Servo pointer;

    private double targetHeading = 0.0;
    private boolean holdingHeading = false;

    private static final double correctionPower = 2.0;
    private static final double allowable = 0.02;
    private static final double deadzoneTurn = 0.05;

    private static final double hiveX = 64.3;
    private static final double hiveY = 58.8;
    private static final double servoOffset = Math.toRadians(80.0);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        pointer = hardwareMap.get(Servo.class, "pointer");
    }

    @Override
    public void start() {
        targetHeading = follower.pose().heading();
    }

    @Override
    public void loop() {
        double turnPower;
        double deadzoneStick = 0.05;
        double forward = Math.abs(gamepad1.left_stick_y) > deadzoneStick ? -gamepad1.left_stick_y : 0.0;
        double strafe = Math.abs(gamepad1.left_stick_x) > deadzoneStick ? -gamepad1.left_stick_x : 0.0;
        double turnInput = -gamepad1.right_stick_x;
        boolean facingTarget = gamepad1.left_trigger > 0.1;

        if (gamepad1.a) {
            follower.setX(0);
            follower.setY(0);
            follower.setHeading(0);
            targetHeading = 0.0;
        }

        double currentX = follower.pose().x();
        double currentY = follower.pose().y();
        double currentHeading = follower.pose().heading();

        double hiveAngle = Math.atan2(hiveY - currentY, hiveX - currentX);
        double relativeAngle = normalizeAngle((hiveAngle - currentHeading) - servoOffset);
        double servoPosition = 0.5 + (relativeAngle / Math.PI);

        servoPosition = Range.clip(servoPosition, 0.0, 1.0);
        pointer.setPosition(servoPosition);

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

        follower.manual(forward, strafe, turnPower);

        telemetry.addData("Facing Target (Left Trigger)", facingTarget);
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        follower.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}