package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "basicauton", group = "LinearOpMode")
public class basicauton extends LinearOpMode {

    private final Pose start = new Pose(56, 8, Math.toRadians(90));
    private final Pose point1 = new Pose(56, 39, Math.toRadians(90));
    private final Pose point2 = new Pose(27, 39, Math.toRadians(-180));
    private final Pose point3 = new Pose(84, 39, Math.toRadians(0));
    private final Pose point4 = new Pose(56, 39, Math.toRadians(-180));
    private final Pose point5 = new Pose(56, 8, Math.toRadians(-90));

    private Path path1;
    private Path path2;
    private Path path3;
    private Path path4;
    private Path path5;


    public void buildPaths() {
        path1 = line(start, point1).linear(start.heading(), point1.heading());
        path2 = line(point1, point2).linear(point1.heading(), point2.heading());
        path3 = line(point2, point3).linear(point2.heading(), point3.heading());
        path4 = line(point3, point4).linear(point3.heading(), point4.heading());
        path5 = line(point4, point5).linear(point4.heading(), point5.heading());
    }

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);
        buildPaths();

        telemetry.addData("Status", "Initialized - Ready to run!");
        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;


        follower.follow(path1);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Current Path", "Path 1");
            telemetry.addData("Robot Pose", follower.pose());
            telemetry.update();
        }


        follower.follow(path2);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Current Path", "Path 2");
            telemetry.addData("Robot Pose", follower.pose());
            telemetry.update();
        }


        follower.follow(path3);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Current Path", "Path 3");
            telemetry.addData("Robot Pose", follower.pose());
            telemetry.update();
        }

        follower.follow(path4);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Current Path", "Path 4");
            telemetry.addData("Robot Pose", follower.pose());
            telemetry.update();
        }

        follower.follow(path5);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Current Path", "Path 5");
            telemetry.addData("Robot Pose", follower.pose());
            telemetry.update();

        }

        while (opModeIsActive()) {
            follower.update();

        }
    }
}