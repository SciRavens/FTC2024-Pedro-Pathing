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


@Autonomous(name = "SampleBasketAuto", group = "Examples")
public class SampleBasketAuto extends OpMode {
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
    private final Pose backup1Pose = new Pose(23, 79, Math.toRadians(0));
    private final Pose strafetofirstsamplePose = new Pose(23,121);
    private final Pose gotoscorefirstsamplePose = new Pose(15,130);
    private final Pose backupafterscoringfirstsamplePose = new Pose(17,132);
    private final Pose gotopickupsecondsamplePose = new Pose(26,132);
    private final Pose gotoscoresecondsamplePose = new Pose(15,130);
    private final Pose backupafterscoringsecondsamplePose = new Pose(17,132);
    private final Pose gotopickupthirdsamplePose = new Pose(45.5, 132);
    private final Pose forwardtopickupthirdsamplePose = new Pose(45.5, 135);
    private final Pose gotosorethirdsamplePose = new Pose(15,130);
    private final Pose backupafterscoringthirdsamplePose = new Pose(17,132);
    private final Pose parkPose = new Pose(63,97);
    private final Pose parkcontrolpointPose = new Pose(60,110);

    private Path scoreSpecimen;
    private PathChain backUp, StrafeToFirstSample, GoToDeliverFirstSample, BackUpFromDroppingFirstSample, GoToPickUpSecondSample, GoToDeliverSecondSample, BackUpFromDroppingSecondSample, GoToPickUpThirdSample, GoFowardToPickUpThirdSample, GoToDeliverThirdSample, BackUpFromDroppingThirdSample, GoToPark ;
    PathBuilder builder;

    public void buildPaths() {
        scoreSpecimen = new Path(new BezierLine(new Point(startPose), new Point(chamberPose)));
        scoreSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));
        //path0.setPathEndVelocityConstraint(4);
        backUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(backup1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        StrafeToFirstSample = StrafeToFirstSample();
        GoToDeliverFirstSample = GoToDeliverFirstSample();
        BackUpFromDroppingFirstSample = BackUpFromDroppingFirstSample();
        GoToPickUpSecondSample = GoToPickUpSecondSample();
        GoToDeliverSecondSample = GoToScoreSecondSample();
        BackUpFromDroppingSecondSample = BackUpFromDroppingSecondSample();
        GoToPickUpThirdSample = GoToPickUpThirdSample();
        GoFowardToPickUpThirdSample = GoFowardToPickUpThirdSample();
        GoToDeliverThirdSample = GoToDeliverThirdSample();
        BackUpFromDroppingThirdSample = BackUpFromDroppingThirdSample();
        GoToPark = GoToPark();
    }

    public PathChain StrafeToFirstSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(backup1Pose.getX(), backup1Pose.getY(), Point.CARTESIAN),
                                new Point(strafetofirstsamplePose.getX(), strafetofirstsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return p;
    }

    public PathChain GoToDeliverFirstSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(strafetofirstsamplePose.getX(), strafetofirstsamplePose.getY(), Point.CARTESIAN),
                                new Point(gotoscorefirstsamplePose.getX(), gotoscorefirstsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();
        return p;
    }

    public PathChain BackUpFromDroppingFirstSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(gotoscorefirstsamplePose.getX(), gotoscorefirstsamplePose.getY(), Point.CARTESIAN),
                                new Point(backupafterscoringfirstsamplePose.getX(), backupafterscoringfirstsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();
        return p;
    }

    public PathChain GoToPickUpSecondSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(backupafterscoringfirstsamplePose.getX(), backupafterscoringfirstsamplePose.getY(), Point.CARTESIAN),
                                new Point(gotopickupsecondsamplePose.getX(), gotopickupsecondsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();
        return p;
    }

    public PathChain GoToScoreSecondSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(gotopickupsecondsamplePose.getX(), gotopickupsecondsamplePose.getY(), Point.CARTESIAN),
                                new Point(gotoscoresecondsamplePose.getX(), gotoscoresecondsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();
        return p;
    }

    public PathChain BackUpFromDroppingSecondSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(gotoscoresecondsamplePose.getX(), gotoscoresecondsamplePose.getY(), Point.CARTESIAN),
                                new Point(backupafterscoringsecondsamplePose.getX(),backupafterscoringsecondsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();
        return p;
    }

    public PathChain GoToPickUpThirdSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(backupafterscoringsecondsamplePose.getX(), backupafterscoringsecondsamplePose.getY(), Point.CARTESIAN),
                                new Point(gotopickupthirdsamplePose.getX(), gotopickupthirdsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
        return p;
    }

    public PathChain GoFowardToPickUpThirdSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(gotopickupthirdsamplePose.getX(), gotopickupthirdsamplePose.getY(), Point.CARTESIAN),
                                new Point(forwardtopickupthirdsamplePose.getX(), forwardtopickupthirdsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        return p;
    }

    public PathChain GoToDeliverThirdSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(forwardtopickupthirdsamplePose.getX(), forwardtopickupthirdsamplePose.getY(), Point.CARTESIAN),
                                new Point(gotosorethirdsamplePose.getX(), gotosorethirdsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
        return p;
    }

    public PathChain BackUpFromDroppingThirdSample() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(gotosorethirdsamplePose.getX(), gotosorethirdsamplePose.getY(), Point.CARTESIAN),
                                new Point(backupafterscoringthirdsamplePose.getX(), backupafterscoringthirdsamplePose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();
        return p;
    }

    public PathChain GoToPark() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(backupafterscoringthirdsamplePose.getX(), backupafterscoringthirdsamplePose.getY(), Point.CARTESIAN),
                                new Point(parkcontrolpointPose.getX(), parkcontrolpointPose.getY(), Point.CARTESIAN),
                                new Point(parkPose.getX(), parkPose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        return p;
    }





    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                arm.setPosChamber(false);
                wrist.setPosHighChamber(false);
                slider.HighChamber();
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.setMaxPower(1.0);
                    slider.setPower(0);
                    follower.followPath(scoreSpecimen);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getPose().getX() > (chamberPose.getX() - 1)) {
                    claw.open();
                    follower.followPath(backUp, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.getPose().getX() < (backup1Pose.getX() - 1)) {
                    arm.setPosFold(false);
                    wrist.setPosFold(false);
                    slider.InitialPose();
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    slider.setPower(0);
                    follower.setMaxPower(1.0);
                    follower.followPath(GoToDeliverFirstSample);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // claw and slider stuff here to put the specimen
                    arm.setPosSample(true);
                    claw.open();
                    wrist.setPosSample(false);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    claw.close();
                    setPathState(61);
                }
                break;
            case 61:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    arm.setPosBasket(false);
                    wrist.setPosBasket(false);
                    slider.HighBasket();
                    follower.setMaxPower(0.4);
                    setPathState(62);
                }
                break;
            case 62:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    slider.setPower(0);
                    follower.followPath(GoToDeliverFirstSample);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    // drop the sample
                    claw.open();
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(BackUpFromDroppingFirstSample);
                    setPathState(81);
                }
                break;
            case 81:
                if (!follower.isBusy()) {
                    arm.setPosFold(false);
                    wrist.setPosFold(false);
                    slider.InitialPose(); // Making sound
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    slider.setPower(0);
                    follower.setMaxPower(1.0);
                    follower.followPath(GoToPickUpSecondSample);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    arm.setPosSample(true);
                    wrist.setPosSample(true);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    claw.close();
                    setPathState(111);
                }
                break;
            case 111:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(0.5);
                    arm.setPosBasket(false);
                    wrist.setPosBasket(false);
                    setPathState(12);
                }
                break;
//            case 12:
//                if (pathTimer.getElapsedTimeSeconds() > 1) {
//                    follower.followPath(AlignWithBasketToDeliverSecondSample);
//                    setPathState(121);
//                }
//                break;
            case 12:
                if (!follower.isBusy()) {
                    slider.HighBasket();
                    setPathState(122);
                }
                break;
            case 122:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.setMaxPower(0.5);
                    follower.followPath(GoToDeliverSecondSample);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    claw.open();
                    setPathState(131);
                }
                break;
            case 131:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    // NEED TO FIX THIS... back and then turn
                    follower.setMaxPower(1.0);
                    follower.followPath(BackUpFromDroppingSecondSample);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    slider.InitialPose();
                    setPathState(15);
                }
                break;
//            case 141:
//                if (pathTimer.getElapsedTimeSeconds() > 1) {
//                    follower.setMaxPower(0.5);
//                    follower.followPath(AlignWithThirdSample);
//                    setPathState(15);
//                }
//                break;
            case 15:
                if(!follower.isBusy()) {
                    arm.setPosSample(false);
                    wrist.setPosSample(false);
                    clawAngle.setVertical();
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(GoToPickUpThirdSample);
                    setPathState(161);
                }
                break;
            case 161:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(GoFowardToPickUpThirdSample);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    claw.close();
//                    follower.followPath(BackUpFromPickingTheThirdSampleUp);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(GoToDeliverThirdSample);
                    slider.HighBasket();
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    // SLIDER NEED TO GO UP...
                    claw.open();
                    setPathState(191);
                }
                break;
            case 191:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    // Backup here
                    follower.followPath(BackUpFromDroppingThirdSample);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    slider.InitialPose();
                    arm.setPosFold(false);
                    wrist.setPosFold(false);
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(1.0);
                    arm.setPosPark(true);
                    wrist.setPosPark(true);
                    follower.followPath(GoToPark);
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
        follower.setMaxPower(0.4);

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

