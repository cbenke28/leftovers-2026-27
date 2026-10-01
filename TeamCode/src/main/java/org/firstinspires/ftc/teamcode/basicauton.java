package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "basicauton", group = "LinearOpMode")
//public class basicauton extends LinearOpMode {
//
//    private Follower follower;
//    private final Pose start = new Pose(56, 8, Math.toRadians(90));
//    private final Pose path1Pose = new Pose(56.3137, 35.0588, Math.toRadians(180));
//    private final Pose parkPose = new Pose(135.143, 34.388, Math.toRadians(179.5125));
//    private final Pose point3Pose = new Pose(33.7867, 113.5429, Math.toRadians(142.0117));
//
//    private Path path1;
//    private Path parkPath;
//    private Path path3;
//
//
//    public void buildPaths() {
//        path1 = new Path(new BezierLine(start, path1Pose));
//        path1.setLinearHeadingInterpolation(start.getHeading(), path1Pose.getHeading());
//
//        parkPath = new Path(new BezierLine(path1Pose, parkPose));
//        parkPath.setTangentHeadingInterpolation();
//        parkPath.reverseHeadingInterpolation();
//
//        path3 = new Path(new BezierLine(parkPose, point3Pose));
//        path3.setTangentHeadingInterpolation();
//    }
//
//    @Override
//    public void runOpMode() {
//        // 1. Initialize the Pedro Pathing Follower using your team's Constants helper
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(start);
//
//
//        buildPaths();
//
//        telemetry.addData("Status", "Initialized - Ready to run!");
//        telemetry.update();
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//
//        follower.followPath(path1);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            telemetry.addData("Current Path", "Path 1");
//            telemetry.addData("Robot Pose", follower.getPose());
//            telemetry.update();
//        }
//
//
//        follower.followPath(parkPath);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            telemetry.addData("Current Path", "Park Path");
//            telemetry.addData("Robot Pose", follower.getPose());
//            telemetry.update();
//        }
//
//
//        follower.followPath(path3);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            telemetry.addData("Current Path", "Path 3");
//            telemetry.addData("Robot Pose", follower.getPose());
//            telemetry.update();
//        }
//
//        while (opModeIsActive()) {
//            follower.update();
//
//        }
//    }
//}
