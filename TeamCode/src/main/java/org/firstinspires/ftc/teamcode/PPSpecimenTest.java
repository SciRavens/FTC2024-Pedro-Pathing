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


@Autonomous(name = "PPSpecimenTest", group = "Examples")
public class PPSpecimenTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private final Pose startPose = new Pose(9.757, 65.000, Math.toRadians(0));
    private final Pose chamberPose = new Pose(40, 65.0, Math.toRadians(00));
    private final Pose alignPose = new Pose(35, 65.0, Math.toRadians(00));


    private Path scoreSpecimen, path1, path2, path3;
    private PathChain backUp, pathChain2, pathChain3;
    PathBuilder builder;

    public Robot robot;
    //public DriveTrain DT;
    public Slider slider;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public ClawAngle clawAngle;

    public void buildPaths() {
        scoreSpecimen = new Path(new BezierLine(new Point(startPose), new Point(chamberPose)));
        scoreSpecimen.setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading());
                //path0.setPathEndVelocityConstraint(4);
        backUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(alignPose)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), alignPose.getHeading())
                //.setPathEndVelocityConstraint(4)
                .build();
//        pathChain2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(alignPose), new Point(pose3)))
//                //.setPathEndVelocityConstraint(4)
//                .setLinearHeadingInterpolation(pose3.getHeading(), pose3.getHeading())
//                .build();
//        pathChain3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pose3), new Point(startPose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
//                //.setPathEndVelocityConstraint(4)
//                .build();
    }

//
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Beginning stuff .. raise slider, arm and wrist positions
                slider.HighChamber();
                arm.setChamberPush();
                wrist.setPosHighChamber(false);
                follower.followPath(scoreSpecimen);
                setPathState(1);
                telemetry.addData("PATHSTATE: ", pathState);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the chamberPose's position */
                if(follower.getPose().getX() > (chamberPose.getX() - 1)) {
                    follower.followPath(backUp,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.getPose().getX() < (alignPose.getX() - 1)) {
                    setPathState(-1);
                    // claw and slider stuff here to put the specimen
                    slider.fold();
                    telemetry.addData("PATHSTATE", pathState);
                    break;
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        //pathTimer.resetTimer();
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.5);

        //GeneratedPath();
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
