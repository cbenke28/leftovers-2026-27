package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "limelightTest", group = "Autonomous")
public class limelightTest extends basicauton {

    private Limelight3A limelight;

    private static final double KP_TURN = 0.03;           // Steering gain based on tx (horizontal offset)
    private static final double KP_FORWARD = 0.04;        // Forward gain based on ty error
    private static final double TARGET_TY = 0.0;          // Target vertical offset in degrees
    private static final double MIN_AREA_THRESHOLD = 0.01; // Lower threshold (0.01%) for higher sensitivity
    private static final double MAX_DRIVE_SPEED = 0.5;     // Max speed safety cap

    private int activePipeline = 0;

    public static class TargetData {
        public boolean found = false;
        public double tx = 0.0;
        public double ty = 0.0;
        public double ta = 0.0;
        public String source = "None";
    }

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower from parent basicauton
        follower = Constants.create(hardwareMap);

        // Attempt to find configured Limelight 3A device
        limelight = initLimelight();

        if (limelight == null) {
            while (!isStopRequested() && !opModeIsActive()) {
                telemetry.addData("=== CONFIGURATION ERROR ===", "Limelight3A not found in hardwareMap!");
                telemetry.addLine("Make sure your Ethernet Device configuration is saved on the Driver Station.");
                telemetry.addLine();
                telemetry.addLine("--- Currently Configured Devices in HardwareMap ---");
                try {
                    List<HardwareDevice> allDevices = hardwareMap.getAll(HardwareDevice.class);
                    if (allDevices != null && !allDevices.isEmpty()) {
                        for (HardwareDevice dev : allDevices) {
                            telemetry.addLine("- " + dev.getDeviceName() + " (" + dev.getClass().getSimpleName() + ")");
                        }
                    } else {
                        telemetry.addLine("(No devices found in active hardwareMap)");
                    }
                } catch (Exception e) {
                    telemetry.addLine("Unable to list hardware devices: " + e.getMessage());
                }
                telemetry.update();
                sleep(200);
            }
            return;
        }

        limelight.pipelineSwitch(activePipeline);
        limelight.start();

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        // Interactive pre-start loop: allows switching pipeline and previewing target detection
        while (!isStopRequested() && !opModeIsActive()) {
            // Use Gamepad 1 D-Pad to switch pipelines if needed
            if (gamepad1.dpad_up && !lastDpadUp) {
                activePipeline = (activePipeline + 1) % 10;
                limelight.pipelineSwitch(activePipeline);
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                activePipeline = (activePipeline + 9) % 10;
                limelight.pipelineSwitch(activePipeline);
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            LLResult result = limelight.getLatestResult();
            TargetData target = extractTarget(result);

            LLStatus status = limelight.getStatus();
            telemetry.addData("Limelight Pipeline", "Index: %d (Use Gamepad D-Pad Up/Down to switch)", activePipeline);
            telemetry.addData("Limelight Status", "FPS: %d | Temp: %.1fC", (int) status.getFps(), status.getTemp());

            if (target.found) {
                telemetry.addData("TARGET DETECTED!", "Source: %s", target.source);
                telemetry.addData("Target Coordinates", "tx: %.2f | ty: %.2f | ta: %.2f", target.tx, target.ty, target.ta);
            } else {
                telemetry.addData("Target Status", "No target detected on Pipeline %d. Try switching pipelines with D-Pad!", activePipeline);
            }

            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) return;

        // Drive towards yellow ball using Limelight vision feedback
        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();
            TargetData target = extractTarget(result);

            double forwardPower = 0.0;
            double strafePower = 0.0;
            double turnPower = 0.0;

            if (target.found) {
                // Proportional turn power to center target horizontally
                double calculatedTurn = target.tx * KP_TURN;

                // Proportional forward power to drive toward target
                double tyError = TARGET_TY - target.ty;
                double calculatedForward = tyError * KP_FORWARD;

                // Clamp drive powers to max safe speed limit
                forwardPower = Math.max(-MAX_DRIVE_SPEED, Math.min(MAX_DRIVE_SPEED, calculatedForward));
                turnPower = Math.max(-MAX_DRIVE_SPEED, Math.min(MAX_DRIVE_SPEED, calculatedTurn));

                telemetry.addData("Target Status", "Yellow Ball Detected via %s", target.source);
                telemetry.addData("tx (deg)", "%.2f", target.tx);
                telemetry.addData("ty (deg)", "%.2f", target.ty);
                telemetry.addData("Area (%)", "%.2f", target.ta);
            } else {
                telemetry.addData("Target Status", "No yellow ball detected on Pipeline %d", activePipeline);
            }

            // Drive robot using Pedro Pathing manual drive
            follower.manual(forwardPower, strafePower, turnPower);
            follower.update();

            // Display Limelight system telemetry
            LLStatus status = limelight.getStatus();
            telemetry.addData("Limelight Pipeline", activePipeline);
            telemetry.addData("Limelight Temp", "%.1f C", status.getTemp());
            telemetry.addData("Limelight FPS", status.getFps());
            telemetry.addData("Drive Powers", "Forward: %.2f | Turn: %.2f", forwardPower, turnPower);
            telemetry.update();
        }

        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Comprehensive target extraction from LLResult.
     * Checks Detector results (neural network), Color results (color thresholding),
     * and primary pipeline targeting outputs (tx, ty, ta).
     */
    private TargetData extractTarget(LLResult result) {
        TargetData target = new TargetData();
        if (result == null || !result.isValid()) {
            return target;
        }

        // 1. Check Neural Detector results (e.g. "yellow", "sample", "ball", "element")
        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        if (detectorResults != null && !detectorResults.isEmpty()) {
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                String className = dr.getClassName() != null ? dr.getClassName().toLowerCase() : "";
                if (className.contains("yellow") || className.contains("sample") || className.contains("ball") || className.contains("element") || className.isEmpty()) {
                    target.found = true;
                    target.tx = dr.getTargetXDegrees();
                    target.ty = dr.getTargetYDegrees();
                    target.ta = dr.getTargetArea();
                    target.source = "Detector (" + (dr.getClassName() != null ? dr.getClassName() : "object") + ")";
                    return target;
                }
            }
        }

        // 2. Check Color Thresholding results
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        if (colorResults != null && !colorResults.isEmpty()) {
            LLResultTypes.ColorResult cr = colorResults.get(0);
            target.found = true;
            target.tx = cr.getTargetXDegrees();
            target.ty = cr.getTargetYDegrees();
            target.ta = cr.getTargetArea();
            target.source = "Color Result";
            return target;
        }

        // 3. Fallback to primary pipeline crosshair targeting outputs (tx, ty, ta)
        double ta = result.getTa();
        if (ta >= MIN_AREA_THRESHOLD || Math.abs(result.getTx()) > 0.001 || Math.abs(result.getTy()) > 0.001) {
            target.found = true;
            target.tx = result.getTx();
            target.ty = result.getTy();
            target.ta = ta;
            target.source = "Primary Crosshair (tx/ty)";
            return target;
        }

        return target;
    }

    /**
     * Automatically retrieves the Limelight 3A hardware device from the hardwareMap.
     */
    private Limelight3A initLimelight() {
        // 1. Retrieve all registered Limelight3A instances regardless of configured name
        try {
            List<Limelight3A> allLimelights = hardwareMap.getAll(Limelight3A.class);
            if (allLimelights != null && !allLimelights.isEmpty()) {
                Limelight3A device = allLimelights.get(0);
                telemetry.addData("Limelight Device", "Found via Limelight3A class registration");
                return device;
            }
        } catch (Exception ignored) {}

        // 2. Try explicit common default names ("ethernet device", "Ethernet Device", "limelight", etc.)
        String[] possibleNames = {
            "ethernet device",
            "Ethernet Device",
            "limelight",
            "Limelight",
            "ethernet",
            "Ethernet",
            "limelight3a",
            "Limelight3A",
            "limelight 3a",
            "Limelight 3A"
        };

        for (String name : possibleNames) {
            try {
                Limelight3A ll = hardwareMap.tryGet(Limelight3A.class, name);
                if (ll != null) {
                    telemetry.addData("Limelight Device", "Found with name '%s'", name);
                    return ll;
                }
            } catch (Exception ignored) {}
        }

        // 3. Scan all hardware devices in hardwareMap for any Limelight3A instance
        try {
            List<HardwareDevice> allDevices = hardwareMap.getAll(HardwareDevice.class);
            if (allDevices != null) {
                for (HardwareDevice dev : allDevices) {
                    if (dev instanceof Limelight3A) {
                        telemetry.addData("Limelight Device", "Found scanned device: %s", dev.getDeviceName());
                        return (Limelight3A) dev;
                    }
                }
            }
        } catch (Exception ignored) {}

        return null;
    }

    /**
     * Helper method to seek and drive to a yellow ball within a timeout limit.
     */
    public boolean driveToYellowBall(double timeoutSeconds) {
        if (limelight == null) {
            limelight = initLimelight();
        }

        if (limelight == null) return false;

        limelight.pipelineSwitch(activePipeline);
        limelight.start();

        double startTime = getRuntime();

        while (opModeIsActive() && (getRuntime() - startTime) < timeoutSeconds) {
            LLResult result = limelight.getLatestResult();
            TargetData target = extractTarget(result);

            if (target.found) {
                double turnPower = Math.max(-MAX_DRIVE_SPEED, Math.min(MAX_DRIVE_SPEED, target.tx * KP_TURN));
                double forwardPower = Math.max(-MAX_DRIVE_SPEED, Math.min(MAX_DRIVE_SPEED, (TARGET_TY - target.ty) * KP_FORWARD));

                follower.manual(forwardPower, 0.0, turnPower);
                follower.update();

                // Target reached when centered and at target vertical offset
                if (Math.abs(target.tx) < 2.0 && Math.abs(TARGET_TY - target.ty) < 2.0) {
                    follower.manual(0, 0, 0);
                    follower.update();
                    return true;
                }
            } else {
                follower.manual(0, 0, 0);
                follower.update();
            }
        }

        follower.manual(0, 0, 0);
        follower.update();
        return false;
    }
}
