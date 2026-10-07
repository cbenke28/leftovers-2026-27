package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class cameratracking {

    private Limelight3A limelight;
    private Servo pointerServo;

    // Servo Configuration Parameters
    // Center position of the servo (0.5 is default center for 0 to 1 range)
    private static final double SERVO_CENTER_POS = 0.5;

    // Total mechanical range of the servo in degrees (typically 180 or 270 degrees)
    private static final double SERVO_RANGE_DEGREES = 180.0;

    // Minimum and Maximum allowed servo positions to protect mechanical stops
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // Proportional Gain for closed-loop tracking (adjust during tuning)
    private static final double KP_TRACKING = 0.003;

    // Current target servo position
    private double currentServoPos = SERVO_CENTER_POS;

    /**
     * Initializes the Limelight and Servo hardware devices.
     *
     * @param hardwareMap FTC HardwareMap from the active OpMode
     */
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pointerServo = hardwareMap.get(Servo.class, "pointerServo");

        // Set initial pointer position to center
        pointerServo.setPosition(SERVO_CENTER_POS);

        // Switch to pipeline 0 (e.g. AprilTag or Color detection pipeline configured in Limelight)
        limelight.pipelineSwitch(0);

        // Start receiving frames from Limelight
        limelight.start();
    }

    /**
     * Sets the active Limelight pipeline index.
     *
     * @param pipelineIndex Index of pipeline created in Limelight web interface
     */
    public void setPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    /**
     * Direct Feed-Forward Tracking Mode (Camera Fixed to Robot Chassis):
     * Maps Limelight horizontal offset angle (tx) directly to the servo angle.
     *
     * @param telemetry Telemetry instance for outputting debug data
     */
    public void updateDirectTracking(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // tx is horizontal offset in degrees from crosshair (-31.85 to +31.85 deg)
            double tx = result.getTx();
            double ty = result.getTy();

            // Convert offset angle in degrees to servo position offset (0.0 - 1.0 scale)
            // Example: +10 deg error -> offset = +10 / 180 = +0.055 position change
            double posOffset = tx / SERVO_RANGE_DEGREES;
            double targetPos = SERVO_CENTER_POS + posOffset;

            // Clamp servo position to safe operating range
            currentServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, targetPos));
            pointerServo.setPosition(currentServoPos);

            if (telemetry != null) {
                telemetry.addData("Limelight", "Target Detected");
                telemetry.addData("Target tx (deg)", "%.2f", tx);
                telemetry.addData("Target ty (deg)", "%.2f", ty);
                telemetry.addData("Servo Position", "%.3f", currentServoPos);
            }
        } else {
            // Keep current position or return to center when target is lost
            if (telemetry != null) {
                telemetry.addData("Limelight", "No Valid Target");
            }
        }
    }

    /**
     * Closed-Loop / Continuous Tracking Mode (Camera Mounted on Servo/Turret):
     * Uses Proportional control to iteratively steer servo until tx reaches zero.
     *
     * @param telemetry Telemetry instance for debug logging
     */
    public void updateClosedLoopTracking(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            // Proportional correction: shift servo position by (Kp * tx)
            currentServoPos += (tx * KP_TRACKING);

            // Clamp within limits
            currentServoPos = Math.max(SERVO_MIN_POS, Math.min(SERVO_MAX_POS, currentServoPos));
            pointerServo.setPosition(currentServoPos);

            if (telemetry != null) {
                telemetry.addData("Limelight Closed-Loop", "Target Acquired");
                telemetry.addData("tx Error", "%.2f deg", tx);
                telemetry.addData("Pointer Pos", "%.3f", currentServoPos);
            }
        } else {
            if (telemetry != null) {
                telemetry.addData("Limelight Closed-Loop", "Searching...");
            }
        }
    }

    /**
     * Stops the Limelight vision sensor. Should be called at end of OpMode.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
