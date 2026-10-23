package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Autonomous OpMode using FTC Limelight 3A to track and drive to "pollens".
 */
@Autonomous(name = "limelightTest", group = "Autonomous")
public class limelightTest extends basicauton {

    // --- Control Tuning Parameters ---
    private static final double KP_AIM = 0.025;
    private static final double KP_DISTANCE = 0.06;
    private static final double DESIRED_AREA = 8.0;
    private static final double MAX_DRIVE_POWER = 0.4;
    private static final double MAX_TURN_POWER = 0.4;

    // Must match the pipeline index tuned in the Limelight web browser interface!
    private static final int PIPELINE_INDEX = 0;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();

        telemetry.addData("Limelight Status", "Initialized - Waiting for Start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            double drivePower = 0.0;
            double turnPower = 0.0;

            // Diagnostic Telemetry
            if (status != null) {
                telemetry.addData("Limelight Hardware", "FPS: " + status.getFps() + " | Pipeline: " + status.getPipelineIndex() + " (" + status.getPipelineType() + ")");
            }

            if (result == null) {
                telemetry.addData("Network Status", "No data received from Limelight yet");
            } else if (!result.isValid()) {
                telemetry.addData("Network Status", "Data received, but NO target matched on Pipeline " + PIPELINE_INDEX);
            } else {
                telemetry.addData("Network Status", "TARGET DETECTED!");

                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                // Steering Control
                turnPower = tx * KP_AIM;
                turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));

                // Distance Control
                double areaError = DESIRED_AREA - ta;
                if (areaError > 0.5) {
                    drivePower = areaError * KP_DISTANCE;
                    drivePower = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, drivePower));
                } else {
                    drivePower = 0.0;
                }

                telemetry.addData("tx (Aim Error)", tx);
                telemetry.addData("ty (Elevation)", ty);
                telemetry.addData("ta (Target Area)", ta);
                telemetry.addData("Drive Power", drivePower);
                telemetry.addData("Turn Power", turnPower);
            }

            follower.manual(drivePower, 0.0, turnPower);
            follower.update();

            telemetry.update();
        }

        limelight.stop();
    }
}
