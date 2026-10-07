package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Example TeleOp demonstrating driving around with Mecanum drive while
 * the camera tracking class keeps the laser pointer / servo pointed at the Hive.
 */
@TeleOp(name="Limelight Pointer TeleOp", group="LinearOpmode")
public class LimelightPointerTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Drivetrain Hardware Setup
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Camera Tracking helper
        cameratracking tracker = new cameratracking();
        tracker.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Pointing ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Drivetrain Control (Mecanum Drive)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            leftFront.setPower((y + x + rx) / denominator);
            leftBack.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);

            // 2. Update Pointer Tracking (Direct Tracking Mode)
            // (Use tracker.updateClosedLoopTracking(telemetry) if camera is mounted on the servo)
            tracker.updateDirectTracking(telemetry);

            telemetry.update();
        }

        // Clean up Limelight resources on stop
        tracker.stop();
    }
}
