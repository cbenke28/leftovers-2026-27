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



    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);


    }

    @Override
    public void loop() {
        follower.manual(
        //ManualDrive.driveOrHold(
                //follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x * 0.8,
                -gamepad1.right_stick_x
        );

        if (gamepad1.right_trigger_pressed) {intake.setPower(1.0);}
        else {intake.setPower(0.0);}
        //intake motor


        telemetry.addData("Botpose", follower.pose());
        follower.update();
    }
}