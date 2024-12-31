package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.jvm.Gen;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;


@Autonomous(name = "PPSpecimenTest", group = "Examples")
public class PPSpecimenTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private final Pose startPose = new Pose(9.757, 65.000, Math.toRadians(0));
    private final Pose chamberPose = new Pose(42.5, 65.0, Math.toRadians(00));
    private final Pose alignPose = new Pose(30, 65.0, Math.toRadians(00));
    private final Pose strafePose = new Pose(30, 32.0, Math.toRadians(00));



    private Path scoreSpecimen, path1, path2, path3;
    private PathChain backUp, pathChain2, pathChain3;
    private PathChain strafeToSamples;
    PathBuilder builder;

    public Robot robot;
    //public DriveTrain DT;
    public Slider slider;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public ClawAngle clawAngle;
    private boolean initSliders_done = false;
    private PathChain pushSamplesPathChain;
    public void buildPaths() {
        scoreSpecimen = new Path(new BezierLine(new Point(startPose), new Point(chamberPose)));
        scoreSpecimen.setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading());
                //path0.setPathEndVelocityConstraint(4);
        backUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(alignPose)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), alignPose.getHeading()).build();

        strafeToSamples = follower.pathBuilder()
                .addPath(new BezierLine(new Point(alignPose), new Point(strafePose)))
                .setLinearHeadingInterpolation(alignPose.getHeading(), strafePose.getHeading())
                //.setPathEndVelocityConstraint(4)
                .build();

        pushSamplesPathChain = GeneratPushSamplesPath();

    }

    public PathChain GeneratPushSamplesPath() {
        builder = new PathBuilder();
        builder.addPath(
                        // Line 2
                        new BezierLine(
                                new Point(alignPose.getX(), alignPose.getY(), Point.CARTESIAN),
                                new Point(alignPose.getX(), 35.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(alignPose.getX(), 35.000, Point.CARTESIAN),
                                new Point(60.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(60.000, 35.000, Point.CARTESIAN),
                                new Point(60.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(60.000, 25.000, Point.CARTESIAN),
                                new Point(20.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(20.000, 25.000, Point.CARTESIAN),
                                new Point(25.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

//                .addPath(
//                        // Line 6
//                        new BezierLine(
//                                new Point(20.000, 25.000, Point.CARTESIAN),
//                                new Point(60.000, 25.000, Point.CARTESIAN)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .addPath(
//                        // Line 7
//                        new BezierLine(
//                                new Point(60.000, 25.000, Point.CARTESIAN),
//                                new Point(60.000, 15.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 8
//                        new BezierLine(
//                                new Point(60.000, 15.000, Point.CARTESIAN),
//                                new Point(20.000, 15.000, Point.CARTESIAN)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .setReversed(true)
//                .addPath(
//                        // Line 9
//                        new BezierLine(
//                                new Point(20.000, 15.000, Point.CARTESIAN),
//                                new Point(60.000, 15.000, Point.CARTESIAN)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .addPath(
//                        // Line 10
//                        new BezierLine(
//                                new Point(60.000, 15.000, Point.CARTESIAN),
//                                new Point(60.000, 10.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 11
//                        new BezierLine(
//                                new Point(60.000, 10.000, Point.CARTESIAN),
//                                new Point(20.000, 10.000, Point.CARTESIAN)
//                        )
//                )
                .setTangentHeadingInterpolation()
                .setReversed(true);

        return builder.build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!initSliders_done) {
                    // Set the Arm, Wrist, Slider ready for pushing specimen
                    arm.setPosChamber(false);
                    wrist.setPosHighChamber(false);
                    slider.HighChamber();
                    initSliders_done = true;
                }
                //Now wait for 1sec
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(1);
                }
                break;
            case 1:
                follower.followPath(scoreSpecimen);
                setPathState(2);
                telemetry.addData("PATHSTATE: ", pathState);
                break;
            case 2:
                if(follower.getPose().getX() > (chamberPose.getX() - 1)) {
                    // Now we pushed it, release the claw and backup
                    claw.open();
                    follower.followPath(backUp,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(follower.getPose().getX() < (alignPose.getX() - 1)) {
                    // We are now at the align pose, so fold all things and move to next path
                    arm.setPosFold(false);
                    wrist.setPosFold(false);
                    slider.InitialPose();
                   // wait for 1 sec to fold
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(pushSamplesPathChain);
                        setPathState(4);
                    }
                }
                telemetry.addData("PATHSTATE", pathState);
                break;
            case 4:
                if(!follower.isBusy()) {
                    // claw and slider stuff here to put the specimen
                    telemetry.addData("PATHSTATE", pathState);
                    arm.setPosSpecimen(false);
                    wrist.setPosSpecimen(false);
                    claw.open();
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    telemetry.addData("PATHSTATE", pathState);
                    claw.close();
                    setPathState(-1);
                }
                break;
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

        arm.setPosStarting(false);
        wrist.setPosStarting(false);
        clawAngle.setHorizontal();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
