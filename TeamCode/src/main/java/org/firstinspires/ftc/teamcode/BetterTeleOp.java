package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "BetterTeleOp")
public class BetterTeleOp extends OpMode {

    private Follower follower;
    private DcMotor intake;

    private double targetHeading = 0.0;
    private boolean holdingHeading = false;

    private boolean headingLockEnabled = true;
    private boolean lastLeftBumper = false;

    private static final double HEADING_P = 2.0;
    private static final double DEADBAND = 0.02;
    private static final double TURN_DEADZONE = 0.05;

    private static final double TARGET_X = 72.0;
    private static final double TARGET_Y = 72.0;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void start() {

        targetHeading = follower.pose().heading();

    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper && !lastLeftBumper) {

            headingLockEnabled = !headingLockEnabled;

            if (headingLockEnabled) {

                targetHeading = follower.pose().heading();

            }

        }
        lastLeftBumper = gamepad1.left_bumper;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;

        double turnPower;


        boolean facingTarget = gamepad1.left_trigger > 0.1;

        if (facingTarget) {

            double currentX = follower.pose().x();
            double currentY = follower.pose().y();


            double deltaX = TARGET_X - currentX;
            double deltaY = TARGET_Y - currentY;
            targetHeading = Math.atan2(deltaY, deltaX);

            double currentHeading = follower.pose().heading();
            double error = normalizeAngle(targetHeading - currentHeading);

            if (Math.abs(error) < DEADBAND) {

                turnPower = 0.0;

            }

            else {

                turnPower = error * HEADING_P;
                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

            }

            holdingHeading = false;

        }

        else if (!headingLockEnabled || Math.abs(turnInput) > TURN_DEADZONE) {

            turnPower = turnInput;
            targetHeading = follower.pose().heading();
            holdingHeading = false;

        }

        else {

            if (!holdingHeading) {

                targetHeading = follower.pose().heading();
                holdingHeading = true;

            }

            double currentHeading = follower.pose().heading();
            double error = normalizeAngle(targetHeading - currentHeading);

            if (Math.abs(error) < DEADBAND) {

                turnPower = 0.0;

            }

            else {

                turnPower = error * HEADING_P;
                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

            }

        }

        follower.manual(forward, strafe, turnPower);

        if (gamepad1.right_trigger > 0.1) {

            intake.setPower(1.0);

        }

        else {

            intake.setPower(0.0);

        }

        telemetry.addData("Facing Target (Left Trigger)", facingTarget);
        telemetry.addData("Heading Lock", headingLockEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turn Power", turnPower);
        follower.update();

    }

    private double normalizeAngle(double angle) {

        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;

    }

}