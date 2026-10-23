package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "BetterTeleOp")
public class BetterTeleOp extends OpMode {

    private Follower follower;
    private DcMotor intake;

    private double targetHeading = 0.0;
    private boolean isHoldingHeading = false;

    private static final double HEADING_P = 2.0;
    private static final double DEADBAND = 0.02;
    private static final double TURN_DEADZONE = 0.05;



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
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;

        double turnPower;


        if (Math.abs(turnInput) > TURN_DEADZONE) {
            turnPower = turnInput;

            targetHeading = follower.pose().heading();
            isHoldingHeading = false;
        } else {

            if (!isHoldingHeading) {
                targetHeading = follower.pose().heading();
                isHoldingHeading = true;
            }


            double currentHeading = follower.pose().heading();
            double error = targetHeading - currentHeading;

            error = normalizeAngle(error);


            if (Math.abs(error) < DEADBAND) {
                turnPower = 0.0;
            } else {
                turnPower = error * HEADING_P;

                turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
            }
        }


        follower.manual(forward, strafe, turnPower);


        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }

        // Telemetry for debugging tuning values
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