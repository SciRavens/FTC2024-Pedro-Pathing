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


@Autonomous(name = "SpecimenAuto2", group = "Examples")
public class SpecimenAuto2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private final Pose startPose = new Pose(9.757, 65.000, Math.toRadians(0)); // starting position
    private final Pose chamberPose = new Pose(42.5, 64.0, Math.toRadians(0));
    private final Pose backup1Pose = new Pose(30, 65.0, Math.toRadians(0));
    private final Pose push1Pose = new Pose(19.0, 26.0, Math.toRadians(0));
    private final Pose push2Pose = new Pose(65.0, 42.0, Math.toRadians(0));
    private final Pose push3Pose = new Pose(55.000, 24.000, Math.toRadians(0));
    private final Pose push4Pose = new Pose(21.000, 24.000, Math.toRadians(0));
    private final Pose push5Pose = new Pose(53.000, 24.000, Math.toRadians(0));
    private final Pose push6Pose = new Pose(53.000, 13.000, Math.toRadians(0));
    private final Pose push7Pose = new Pose(21.000, 13.000, Math.toRadians(0));
    private final Pose push8Pose = new Pose(24.000, 13.000, Math.toRadians(0));
    private final Pose alignwithsecondspecimenPose = new Pose(24, 24, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(22, 24, Math.toRadians(0));
    private final Pose deliversecondspecimenPose = new Pose(42.5, 67.5,Math.toRadians(0));
    private final Pose deliversecondspecimencontrolpoint = new Pose(20, 69,Math.toRadians(0));



    private Path scoreSpecimen;
    private PathChain backUp, pushSamplesPathChain, IntakeSecondSpecimen, DeliverSecondSpecimen, backupGotoThirdSpecimen, push3Specimen;
    PathBuilder builder;

    public Robot robot;
    public Slider slider;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public ClawAngle clawAngle;
    private boolean initSliders_done = false;

    public void buildPaths() {
        scoreSpecimen = new Path(new BezierLine(new Point(startPose), new Point(chamberPose)));
        scoreSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));
        //path0.setPathEndVelocityConstraint(4);
        backUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(backup1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pushSamplesPathChain = GeneratePushSamplesPath();
        IntakeSecondSpecimen = IntakeSecondSpecimen();
        DeliverSecondSpecimen = DeliverSecondSpecimen();
    }


    public PathChain GeneratePushSamplesPath() {
        PathChain p = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(backup1Pose.getX(), backup1Pose.getY(), Point.CARTESIAN),
                                new Point(push1Pose.getX(), push1Pose.getY(), Point.CARTESIAN),
                                new Point(push2Pose.getX(), push2Pose.getY(), Point.CARTESIAN),
                                new Point(push3Pose.getX(), push3Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Point(push3Pose.getX(), push3Pose.getY(), Point.CARTESIAN),
                                new Point(push4Pose.getX(), push4Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(push4Pose.getX(), push4Pose.getY(), Point.CARTESIAN),
                                new Point(push5Pose.getX(), push5Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(push5Pose.getX(), push5Pose.getY(), Point.CARTESIAN),
                                new Point(push6Pose.getX(), push6Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(push6Pose.getX(), push6Pose.getY(), Point.CARTESIAN),
                                new Point(push7Pose.getX(), push7Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(push7Pose.getX(), push7Pose.getY(), Point.CARTESIAN),
                                new Point(push8Pose.getX(), push8Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(push8Pose.getX(), push8Pose.getY(), Point.CARTESIAN),
                                new Point(alignwithsecondspecimenPose.getX(), alignwithsecondspecimenPose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        return p;
    }

    public PathChain IntakeSecondSpecimen() {
        // Change this to a path
        PathChain p = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(alignwithsecondspecimenPose.getX(), alignwithsecondspecimenPose.getY(), Point.CARTESIAN),
                                new Point(pickup2Pose.getX(), pickup2Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        return p;
    }

    public PathChain DeliverSecondSpecimen() {
        PathChain p = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(pickup2Pose.getX(), pickup2Pose.getY(), Point.CARTESIAN),
                                new Point(deliversecondspecimencontrolpoint.getX(), deliversecondspecimencontrolpoint.getY(), Point.CARTESIAN),
                                new Point(deliversecondspecimenPose.getX(), deliversecondspecimenPose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        return p;
    }


        public void autonomousPathUpdate () {
            switch (pathState) {
                case 0:
                    // Set the Arm, Wrist, Slider ready for pushing specimen
                    arm.setPosChamber(false);
                    wrist.setPosHighChamber(false);
                    slider.HighChamber();
                    setPathState(1);
                    break;
                case 1:
                    //Now wait for 1sec
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(scoreSpecimen);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (follower.getPose().getX() > (chamberPose.getX() - 1)) {
                        // Now we pushed it, release the claw and backup
                        claw.open();
                        follower.followPath(backUp, true);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (follower.getPose().getX() < (backup1Pose.getX() - 1)) {
                        // We are now at the align pose, so fold all things and move to next path
                        arm.setPosFold(false);
                        wrist.setPosFold(false);
                        slider.InitialPose();
                        setPathState(4);
                    }
                    break;
                case 4:
                    // wait for 1 sec to fold
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(pushSamplesPathChain);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        arm.setPosSpecimen(false);
                        wrist.setPosSpecimen(false);
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(IntakeSecondSpecimen);
                        setPathState(7);
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        claw.close();
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        slider.LowChamber();
                        follower.followPath(DeliverSecondSpecimen);
                        arm.setPosChamberBack(false);
                        wrist.setPosChamberBack(false);
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (!follower.isBusy()) {
                        slider.HighChamberBack();
                        setPathState(10);
                    }
                    break;
                case 10:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        claw.open();
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
