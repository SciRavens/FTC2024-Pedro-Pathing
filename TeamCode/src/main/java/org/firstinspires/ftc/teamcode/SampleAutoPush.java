package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;


@Autonomous(name = "SampleAutoPush", group = "Examples")
public class SampleAutoPush extends OpMode {
    public Robot robot;
    public Claw claw;
    public Arm arm;
    public Wrist wrist;
    public Slider slider;
    public ClawAngle clawAngle;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private final Pose startPose = new Pose(9.757, 79, Math.toRadians(0));
    private final Pose chamberPose = new Pose(41.5, 79, Math.toRadians(00));
    private final Pose backup1Pose = new Pose(23.25, 79, Math.toRadians(0));
    private final Pose strafetofirstsamplePose = new Pose(72, 120, Math.toRadians(0));
    private final Pose strafetofirstsampleControlPoint = new Pose(1.5, 86, Math.toRadians(0));
    private final Pose pushfirstsamplePose = new Pose(8.000,120.000, Math.toRadians(0));
    private final Pose strafetosecondsamplePose = new Pose(72, 132, Math.toRadians(0));
    private final Pose pushsecondsamplePose = new Pose(8,132, Math.toRadians(0));
    private final Pose strafetothirdsamplePose = new Pose(72, 138, Math.toRadians(0));
    private final Pose pushthirdsamplePose = new Pose(8,138, Math.toRadians(0));






    private Path scoreSpecimen;
    private PathChain backUp, PushSamplesPathChain, GoToPark;
    PathBuilder builder;

    public void buildPaths() {
        scoreSpecimen = new Path(new BezierLine(new Point(startPose), new Point(chamberPose)));
        scoreSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));
        //path0.setPathEndVelocityConstraint(4);
        backUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(backup1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PushSamplesPathChain = PushSamplesPathChain();
        GoToPark = GoToPark();
    }


    public PathChain PushSamplesPathChain() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(backup1Pose.getX(), backup1Pose.getY(), Point.CARTESIAN),
                                new Point(strafetofirstsampleControlPoint.getX(), strafetofirstsampleControlPoint.getY(), Point.CARTESIAN),
                                new Point(strafetofirstsamplePose.getX(), strafetofirstsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                /*
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(strafetofirstsamplePose.getX(), strafetofirstsamplePose.getY(), Point.CARTESIAN),
                                new Point(pushfirstsamplePose.getX(), pushfirstsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(pushfirstsamplePose.getX(), pushfirstsamplePose.getY(), Point.CARTESIAN),
                                new Point(strafetofirstsamplePose.getX(), strafetofirstsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(strafetofirstsamplePose.getX(), strafetofirstsamplePose.getY(), Point.CARTESIAN),
                                new Point(strafetosecondsamplePose.getX(), strafetosecondsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(strafetosecondsamplePose.getX(), strafetosecondsamplePose.getY(), Point.CARTESIAN),
                                new Point(pushsecondsamplePose.getX(), pushsecondsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(pushsecondsamplePose.getX(), pushsecondsamplePose.getY(), Point.CARTESIAN),
                                new Point(strafetosecondsamplePose.getX(), strafetosecondsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(strafetosecondsamplePose.getX(), strafetosecondsamplePose.getY(), Point.CARTESIAN),
                                new Point(strafetothirdsamplePose.getX(), strafetothirdsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(strafetothirdsamplePose.getX(), strafetothirdsamplePose.getY(), Point.CARTESIAN),
                                new Point(pushthirdsamplePose.getX(), pushthirdsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                /*
                 */
                //.setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        return p;
    }

    public PathChain GoToPark() {
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(pushthirdsamplePose.getX(), pushthirdsamplePose.getY(), Point.CARTESIAN),
                                new Point(72.000, 96.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
        return p;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
//                arm.setPosChamber(false);
//                wrist.setPosHighChamber(false);
//                slider.HighChamber();
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.setMaxPower(0.4);
//                    slider.setPower(0);
                    follower.followPath(scoreSpecimen);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getPose().getX() > (chamberPose.getX() - 1)) {
//                    claw.open();
                    follower.followPath(backUp, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.getPose().getX() < (backup1Pose.getX() - 1)) {
//                    arm.setPosFold(false);
//                    wrist.setPosFold(false);
//                    slider.InitialPose();
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(PushSamplesPathChain);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(GoToPark);
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
        slider.autoOpCompletionCheck();
        arm.operate();
        wrist.operate();
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
        claw.close();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.6);

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
