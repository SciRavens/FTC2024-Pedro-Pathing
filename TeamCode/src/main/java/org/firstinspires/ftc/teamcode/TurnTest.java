package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "TurnTest", group = "Examples")
public class TurnTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private final Pose pose0 = new Pose(0, 0, Math.toRadians(0));
    private final Pose pose1 = new Pose(0, 0, Math.toRadians(90));
    private final Pose pose2 = new Pose(0, 0, Math.toRadians(90));
    private final Pose pose3 = new Pose(0, 20, Math.toRadians(90));

    private Path path0, path1, path2, path3;
    private PathChain pathChain1, pathChain2, pathChain3;

    public void buildPaths() {
        path0 = new Path(new BezierLine(new Point(pose0), new Point(pose1)));
        path0.setTangentHeadingInterpolation();
                //path0.setPathEndVelocityConstraint(4);
        pathChain1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose1), new Point(pose2)))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                //.setPathEndVelocityConstraint(4)
                .build();
        pathChain2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose2), new Point(pose3)))
                //.setPathEndVelocityConstraint(4)
                .setLinearHeadingInterpolation(pose3.getHeading(), pose3.getHeading())
                .build();
        pathChain3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose3), new Point(pose0)))
                .setLinearHeadingInterpolation(pose0.getHeading(), pose0.getHeading())
                //.setPathEndVelocityConstraint(4)
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        //pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path0);
                setPathState(1);
                telemetry.addData("PATHSTATE: ", pathState);
                break;
            case 1:
                if(follower.getPose().getX() > (pose1.getX() - 1) && follower.getPose().getY() > (pose1.getY() - 1)) {
                    follower.followPath(pathChain1,true);
                    setPathState(-1);
                    telemetry.addData("PATHSTATE: ", pathState);
                }
                break;
            case 2:
                if(follower.getPose().getX() > (pose2.getX() - 1) && follower.getPose().getY() > (pose2.getY() - 1)) {
                    follower.followPath(pathChain2,true);
                    setPathState(3);
                    telemetry.addData("PATHSTATE: ", pathState);
                }
                break;
            case 3:
                if(follower.getPose().getX() > (pose3.getX() - 1) && follower.getPose().getY() > (pose3.getY() - 1)) {
                    follower.followPath(pathChain3,true);
                    setPathState(4);
                    telemetry.addData("PATHSTATE: ", pathState);
                }
                break;
            case 4:
                if(follower.getPose().getX() > (pose0.getX() - 1) && follower.getPose().getY() > (pose0.getY() - 1)) {
                    setPathState(-1);
                    telemetry.addData("PATHSTATE: ", pathState);
                }
                break;
        }
        telemetry.addData("Current X: ", follower.getPose().getX());
        telemetry.addData("Target X: ", pose1.getX());
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
        follower.setMaxPower(0.5);
        follower.setStartingPose(pose0);

        buildPaths();
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
