package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;



import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "SciRavens-TeleOp")
public class RobotTeleop extends LinearOpMode {
    public Robot robot;
    //public DriveTrain DT;
    public Slider slider;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public ClawAngle clawAngle;
    private Follower follower;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private double slider_pos;
    private static final double Kp = 0.02;
    private static final double Ki = 0.0;
    private static final double Kd = 0.002;

    double strafeHeading; // for strafing straight
    boolean strafeHeadingOn = false;

    private static double Kp_strafing = 0.002;
    private static double DEAD_ZONE = 0.1;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive();
        slider = new Slider(robot);
        arm = new Arm(robot);
        wrist = new Wrist(robot);
        claw = new Claw(robot);
        clawAngle = new ClawAngle(robot);

        arm.setPosStarting(false);
        wrist.setPosStarting(false);
        clawAngle.setHorizontal();
        waitForStart();
        while (opModeIsActive()) {
            follower_operate();
            arm.operate();
            wrist.operate();
            slider_joystick();
            arm_wrist_operate();
            claw_operate();
            robot.telemetry.update();
        }
    }


    private void follower_operate() {
        boolean strafeOnly = false;
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? gamepad1.left_stick_y : 0;

        if (gamepad1.right_trigger > 0.5) {
            follower.setMaxPower(0.25);
        } else if (gamepad1.left_trigger > 0.5) {
            strafeOnly = true;
            if (!strafeHeadingOn) {
                strafeHeading = follower.getPose().getHeading();
            }
            // Heading correction
            double imuHeading = follower.getPose().getHeading();
            double headingError = strafeHeading - imuHeading; // Target heading is initialized when strafing starts
            double correction = Kp_strafing * headingError;

            // Adjust movement with IMU and drift correction
            follower.setTeleOpMovementVectors(xInput, yInput, correction);
        } else {
            follower.setMaxPower(1.0);
        }
        if (!strafeOnly) {
            strafeHeadingOn = false;
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
        follower.update();
    }

    private void arm_wrist_operate() {
        if (gamepad2.dpad_down) {
            arm.setPosSampleTwo(false);
            wrist.setPosSampleTwo(false);
        } else if (gamepad2.y) {
            arm.setPosBasket(false);
            wrist.setPosBasket(false);
            clawAngle.setHorizontal();
            slider.HighBasket();
        } else if (gamepad2.x) {
            clawAngle.setHorizontal();
            arm.setPosStarting(false);
            wrist.setPosStarting(false);
            slider.InitialPose();
        } else if (gamepad2.b) {
            arm.setPosSpecimen(false);
            wrist.setPosSpecimen(false);
        } else if (gamepad2.a) {
            arm.setPosSample(false);
            wrist.setPosSample(false);
        } else if (gamepad2.dpad_left) {
            clawAngle.setHorizontal();
        } else if (gamepad2.dpad_right) {
            clawAngle.setVertical();
        } else if (gamepad2.dpad_up) {
            slider.LowChamber();
            arm.setPosChamberBack(false);
            wrist.setPosChamberBack(false);
        }
    }


    public void get_ticks() {
        slider_pos = slider.getCurrentPosition();
        robot.telemetry.addData("Slider Curr tick:", slider_pos);
    }


    public void slider_operate() {
        slider.autoOpCompletionCheck();
        if (gamepad2.dpad_down) {
            // Go to Low Chamber
            slider.LowChamber();
        } else if (gamepad2.dpad_up) {
            // Go to High Basket
            slider.HighBasket();
        } else if (gamepad2.dpad_right) {
            // Move to Low Basket
            slider.LowBasket();
        } else if (gamepad2.dpad_left) {
            // Move to High Chamber
            slider.HighChamber();
        }
    }

    public void slider_joystick() {
        if (gamepad2.left_stick_y != 0) {
            slider.manualOp(gamepad2.left_stick_y);
        } else {
            slider.manualDefaultStop();
        }
    }
    //    private void slider_pos() {
//        if (gamepad2.dpad_up) {
//            slider.LowBasket();
//        } else if (gamepad2.dpad_down) {
//            slider.LowChamber();
//        } else if (gamepad2.dpad_left) {
//            slider.HighBasket();
//        } else if (gamepad2.dpad_right) {
//            slider.HighChamber();
//        }
//
//    }

    private void claw_operate() {
        if (gamepad2.left_trigger > 0.9) {
            claw.open();
        } else if (gamepad2.right_trigger > 0.9) {
            claw.open();
        } else {
            claw.close();
        }
    }

    private void turnToAngle(double targetAngle) {
        double currentAngle = getHeading(); // Get initial heading
        double error, lastError = 0, totalError = 0;
        double turnPower;
        double maxTurnPower = 0.5; // Cap rotational power to ensure stability

        // Calculate the target heading (in radians)
        double targetHeading = Math.toRadians(targetAngle) + currentAngle;

        while (opModeIsActive()) {
            // Get the current heading
            currentAngle = getHeading();

            // Calculate error between target and current heading
            error = targetHeading - currentAngle;

            // Exit loop when the error is within acceptable bounds
            if (Math.abs(error) < Math.toRadians(1.0)) break;

            // PID calculations
            totalError += error; // Integral term
            double derivative = error - lastError; // Derivative term
            turnPower = Kp * error + Ki * totalError + Kd * derivative;

            // Limit the turn power
            turnPower = Math.max(-maxTurnPower, Math.min(maxTurnPower, turnPower));

            // Set movement vectors to rotate the robot
            // `x` and `y` are zero for pure rotation, and `turnPower` is for rotation
            follower.setTeleOpMovementVectors(0, 0, turnPower);

            lastError = error;

            // Debugging information
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Current Heading (deg)", Math.toDegrees(currentAngle));
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }

        // Stop movement by zeroing the vectors
        follower.setTeleOpMovementVectors(0, 0, 0);
    }

    private double getHeading() {
        // Retrieve the heading from Follower's pose
        return follower.getPose().getHeading(); // Assumes heading is in radians
    }
}