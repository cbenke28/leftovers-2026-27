package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "apriltags", group = "Autonomous")
public class apriltags extends basicauton {

    private Limelight3A limelight;

    // Tuning constants
    private static final double KP_TURN = 0.03;      // Turn gain for centering tx
    private static final double KP_FORWARD = 0.04;    // Forward gain for distance/ty
    private static final double TARGET_TY = -5.0;    // Desired vertical angle at shooting distance (adjust as needed)
    private static final double MAX_SPEED = 0.4;       // Speed safety cap

    private static final int DESIRED_TAG_ID = 32;     // Set your target AprilTag ID (e.g., the hive tag)
    private static final int APRIL_TAG_PIPELINE = 1; // Index of your AprilTag pipeline on the Limelight web UI

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower from parent basicauton
        follower = Constants.create(hardwareMap);

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRIL_TAG_PIPELINE);
        limelight.start();

        telemetry.addData("Status", "Limelight Initialized. Ready for Target Lock.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Main Autonomous Targeting Loop
        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();
            
            double forwardPower = 0.0;
            double strafePower  = 0.0;
            double turnPower    = 0.0;
            boolean targetFound = false;

            if (result != null && result.isValid()) {
                // Get all detected AprilTags (Fiducials)
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                
                if (fiducialResults != null && !fiducialResults.isEmpty()) {
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        // Check if this is our desired target tag ID
                        if (fr.getFiducialId() == DESIRED_TAG_ID) {
                            targetFound = true;

                            // tx = horizontal offset (- = left, + = right)
                            // ty = vertical offset
                            double tx = fr.getTargetXDegrees();
                            double ty = fr.getTargetYDegrees();

                            // Proportional control to pivot/turn toward the tag
                            double calculatedTurn = tx * KP_TURN;

                            // Proportional control to drive forward/backward to reach target distance
                            double tyError = TARGET_TY - ty;
                            double calculatedForward = tyError * KP_FORWARD;

                            // Clamp speeds to safety limits
                            turnPower    = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, calculatedTurn));
                            forwardPower = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, calculatedForward));

                            telemetry.addData("Target Locked!", "ID: %d", fr.getFiducialId());
                            telemetry.addData("tx (Turn Error)", "%.2f", tx);
                            telemetry.addData("ty (Distance Error)", "%.2f", ty);
                            break;
                        }
                    }
                }
            }

            if (!targetFound) {
                telemetry.addData("Status", "Searching for Tag ID %d...", DESIRED_TAG_ID);
                // Optional search behavior: spin slowly in place until tag is detected
                turnPower = 0.2; 
                forwardPower = 0.0;
                strafePower = 0.0;
            }

            // Drive robot using Pedro Pathing manual override while tracking target
            follower.manual(forwardPower, strafePower, turnPower);
            follower.update();

            telemetry.addData("Drive Powers", "Fwd: %.2f | Turn: %.2f", forwardPower, turnPower);
            telemetry.update();
        }

        if (limelight != null) {
            limelight.stop();
        }
    }
}
