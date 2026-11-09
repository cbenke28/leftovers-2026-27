package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "limelightTest", group = "Autonomous")
public class limelightTest extends basicauton {

    private Limelight3A limelight;
    private int activePipeline = 0;

    private static final double KP_TURN = -0.03;
    private static final double APPROACH_SPEED = 0.35; // Constant forward drive speed toward pollen/ball
    private static final double MIN_AREA_THRESHOLD = 0.01;
    private static final double MAX_DRIVE_SPEED = 0.5;

    // Smoothing & Anti-jitter variables
    private double smoothForward = 0.0;
    private double smoothTurn = 0.0;
    private static final double SMOOTHING_FACTOR = 0.3; // Lower = smoother, Higher = more responsive
    private static final double DEADBAND_TX = 1.0; // degrees deadband for turning

    // Auto-cycling pipeline variables
    private double lastPipelineSwitchTime = 0.0;
    private static final double PIPELINE_SWITCH_INTERVAL = 0.25; // Switch every 250ms when searching

    public static class TargetData {
        public boolean found = false;
        public double tx = 0.0, ty = 0.0, ta = 0.0;
        public String source = "None";
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        limelight = initLimelight();

        if (limelight == null) {
            while (!isStopRequested() && !opModeIsActive()) {
                telemetry.addData("ERROR", "Limelight3A not found in hardwareMap!");
                telemetry.update();
                sleep(200);
            }
            return;
        }

        limelight.pipelineSwitch(activePipeline);
        limelight.start();

        boolean lastUp = false;
        boolean lastDown = false;

        // Pre-start loop
        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.dpad_up && !lastUp) activePipeline = (activePipeline + 1) % 10;
            if (gamepad1.dpad_down && !lastDown) activePipeline = (activePipeline + 9) % 10;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            limelight.pipelineSwitch(activePipeline);

            TargetData target = extractTarget(limelight.getLatestResult());
            LLStatus status = limelight.getStatus();

            telemetry.addData("Pipeline", activePipeline);
            telemetry.addData("FPS", status.getFps());
            telemetry.addData("Target Found", target.found ? target.source : "None (tx=%.1f, ty=%.1f)", target.tx, target.ty);
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) return;

        lastPipelineSwitchTime = getRuntime();

        // Autonomous loop
        while (opModeIsActive() && !isStopRequested()) {
            TargetData target = extractTarget(limelight.getLatestResult());

            // If no target is found, cycle through pipelines 0, 1, 2, 3
            if (!target.found) {
                if ((getRuntime() - lastPipelineSwitchTime) > PIPELINE_SWITCH_INTERVAL) {
                    activePipeline = (activePipeline + 1) % 4; // Cycles through 0, 1, 2, 3
                    limelight.pipelineSwitch(activePipeline);
                    lastPipelineSwitchTime = getRuntime();
                }
                telemetry.addData("Status", "Searching (Cycling Pipeline %d)...", activePipeline);
            } else {
                // TARGET FOUND! Lock onto this pipeline and drive forward while steering
                telemetry.addData("Tracking", "%s on Pipeline %d (tx: %.2f, ty: %.2f)", target.source, activePipeline, target.tx, target.ty);
            }

            double[] control = computeSmoothedControl(target);
            follower.manual(control[0], 0.0, control[1]);
            follower.update();
            telemetry.update();
        }

        limelight.stop();
    }

    private double[] computeSmoothedControl(TargetData target) {
        double rawForward = 0.0;
        double rawTurn = 0.0;

        if (target.found) {
            rawForward = APPROACH_SPEED; // Always drive forward toward the pollen/ball
            rawTurn = Math.max(-MAX_DRIVE_SPEED, Math.min(MAX_DRIVE_SPEED, target.tx * KP_TURN));

            // Apply deadband to eliminate micro-jitter when centered on the ball
            if (Math.abs(target.tx) < DEADBAND_TX) rawTurn = 0.0;
        }

        // Low-pass filter (exponential smoothing) for silky smooth movement
        smoothForward = smoothForward * (1.0 - SMOOTHING_FACTOR) + rawForward * SMOOTHING_FACTOR;
        smoothTurn = smoothTurn * (1.0 - SMOOTHING_FACTOR) + rawTurn * SMOOTHING_FACTOR;

        return new double[] { smoothForward, smoothTurn };
    }

    private TargetData extractTarget(LLResult result) {
        TargetData t = new TargetData();
        if (result == null || !result.isValid()) return t;

        List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();
        if (detectors != null && !detectors.isEmpty()) {
            LLResultTypes.DetectorResult dr = detectors.get(0);
            t.found = true;
            t.tx = dr.getTargetXDegrees();
            t.ty = dr.getTargetYDegrees();
            t.ta = dr.getTargetArea();
            t.source = "Detector (" + dr.getClassName() + ")";
            return t;
        }

        List<LLResultTypes.ColorResult> colors = result.getColorResults();
        if (colors != null && !colors.isEmpty()) {
            LLResultTypes.ColorResult cr = colors.get(0);
            t.found = true;
            t.tx = cr.getTargetXDegrees();
            t.ty = cr.getTargetYDegrees();
            t.ta = cr.getTargetArea();
            t.source = "Color";
            return t;
        }

        if (result.getTa() >= MIN_AREA_THRESHOLD || result.getTx() != 0 || result.getTy() != 0) {
            t.found = true;
            t.tx = result.getTx();
            t.ty = result.getTy();
            t.ta = result.getTa();
            t.source = "Primary";
            return t;
        }

        return t;
    }

    private Limelight3A initLimelight() {
        try {
            Limelight3A ll = hardwareMap.tryGet(Limelight3A.class, "limelight");
            if (ll != null) return ll;
            List<Limelight3A> all = hardwareMap.getAll(Limelight3A.class);
            if (!all.isEmpty()) return all.get(0);
        } catch (Exception ignored) {}
        return null;
    }

    public boolean driveToYellowBall(double timeoutSeconds) {
        if (limelight == null) limelight = initLimelight();
        if (limelight == null) return false;

        limelight.pipelineSwitch(activePipeline);
        limelight.start();
        double start = getRuntime();
        double lastSwitch = start;

        while (opModeIsActive() && (getRuntime() - start) < timeoutSeconds) {
            TargetData target = extractTarget(limelight.getLatestResult());

            if (!target.found) {
                if ((getRuntime() - lastSwitch) > PIPELINE_SWITCH_INTERVAL) {
                    activePipeline = (activePipeline + 1) % 4; // Cycles 0, 1, 2, 3
                    limelight.pipelineSwitch(activePipeline);
                    lastSwitch = getRuntime();
                }
            }

            double[] control = computeSmoothedControl(target);
            follower.manual(control[0], 0.0, control[1]);
            follower.update();

            // Stop when target area (ta) is large enough (close to pickup) or timeout reached
            if (target.found && target.ta > 15.0) { // adjust area threshold as needed for pickup
                follower.manual(0, 0, 0);
                follower.update();
                return true;
            }
        }
        follower.manual(0, 0, 0);
        follower.update();
        return false;
    }
}
