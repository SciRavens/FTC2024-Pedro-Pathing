package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;


@Autonomous(name = "PPSpec", group = "Examples")
public class PathTest extends OpMode {
    public Robot robot;
    public Claw claw;
    public Arm arm;
    public Wrist wrist;
    public Slider slider;
    public ClawAngle clawAngle;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private final Pose pose0 = new Pose(25, 25, Math.toRadians(0));
    private final Pose pose1 = new Pose(31, 25, Math.toRadians(00));

    private Path path0;
    private PathChain grabSpecimen, pathChain2, pathChain3;
    PathBuilder builder;

    public void buildPaths() {
        path0 = new Path(new BezierLine(new Point(pose0), new Point(pose1)));
        path0.setLinearHeadingInterpolation(pose0.getHeading(), pose1.getHeading());

//        path0 = new Path(new BezierLine(new Point(pose0), new Point(pose1)));
//        path0.setLinearHeadingInterpolation(pose0.getHeading(), pose1.getHeading());
//                //path0.setPathEndVelocityConstraint(4);
//        pathChain1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pose1), new Point(pose2)))
//                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
//                //.setPathEndVelocityConstraint(4)
//                .build();
//        pathChain2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pose2), new Point(pose3)))
//                //.setPathEndVelocityConstraint(4)
//                .setLinearHeadingInterpolation(pose3.getHeading(), pose3.getHeading())
//                .build();
//        pathChain3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pose3), new Point(pose0)))
//                .setLinearHeadingInterpolation(pose0.getHeading(), pose0.getHeading())
//                //.setPathEndVelocityConstraint(4)
//                .build();
            grabSpecimen = GeneratedPath();
    }

    public PathChain GeneratedPath() {
        builder = new PathBuilder();
        builder.addPath(
                        new BezierLine(
                                new Point(pose0.getX(), pose0.getY(), Point.CARTESIAN),
                                new Point(-3, 25.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setReversed(true);
        return builder.build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path0);
                setPathState(1);
                telemetry.addData("PATHSTATE", pathState);
                break;
            case 1:
                if(!follower.isBusy()) {
                    // claw and slider stuff here to put the specimen
                    telemetry.addData("PATHSTATE", pathState);
                    claw.close();
                    setPathState(-1);
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        claw = new Claw(robot);
        wrist = new Wrist(robot);
        arm = new Arm(robot);
        slider = new Slider(robot);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        clawAngle = new ClawAngle(robot);

        arm.setPosSpecimen(false);
        wrist.setPosSpecimen(false);
        claw.open();
        clawAngle.setHorizontal();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(pose0); //IMPORTANT

        follower.setMaxPower(0.5);

        buildPaths();
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
    }

    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
