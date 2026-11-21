package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Limelight Blue", group = "Autonomous")
public class limelightBlue extends limelightTest {
    public limelightBlue() {
        super();
        // Blue Team: Tracks pollen, April tags, and red balls (Pipelines 0, 1, 2)
        setAllowedPipelines(new int[] {0, 1, 2});
    }
}
