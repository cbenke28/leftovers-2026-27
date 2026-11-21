package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Limelight Red", group = "Autonomous")
public class limelightRed extends limelightTest {
    public limelightRed() {
        super();
        // Red Team: Tracks pollen, April tags, and blue balls (Pipelines 0, 1, 3)
        setAllowedPipelines(new int[] {0, 1, 3});
    }
}
