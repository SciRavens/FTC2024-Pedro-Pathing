package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveTrain {
    private Robot robot = null;
    private volatile Gamepad gamepad = null;

    private double normal_speed = 0.8;
    private double medium_speed = 0.3;
    private double slow_speed = 0.2;
    private boolean verbose = false;

    public DriveTrain(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        robot.follower.startTeleopDrive();
    }

    public void stopDrive() {
        robot.follower.setTeleOpMovementVectors(0,0,0);

    }

    private void drive(double left_y, double left_x, double right_x, double speed_factor) {
        double y = -left_y * speed_factor;  // Forward/backward (inverted)
        double x = -left_x * speed_factor;  // Strafe (adjusted for mecanum drift)
        double rx = right_x * speed_factor;  // Rotation
        robot.follower.setTeleOpMovementVectors(y, x, rx);
        robot.follower.update();
    }

    public void drive_normal() {
        drive(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, normal_speed);    }

    public void drive_medium() {
        drive(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, medium_speed);
    }

    public void drive_slow() {
        double x = 0, y = 0;
        if (gamepad.dpad_up) {
            y = -0.2;
        } else if (gamepad.dpad_down) {
            y = 0.2;
        } else if (gamepad.dpad_left) {
            x = 0.36;
        } else if (gamepad.dpad_right) {
            x = -0.36;
        }
        if (x == 0 && y == 0) {
            return;
        }
        drive(y, x, 0.0, normal_speed);
    }
    public void drive() {
        if (gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0 || gamepad.right_stick_x != 0) {
            if (gamepad.left_trigger > 0 || gamepad.right_trigger > 0) {
                drive_slow();
            } else {
                drive_normal();
            }
        } else if (gamepad.dpad_left) {
            //robot.mDrive.turn(Math.toRadians(90));
        } else if (gamepad.dpad_right) {
            //robot.mDrive.turn(Math.toRadians(-90));
        } else if (gamepad.dpad_down || gamepad.dpad_up) {
            //robot.mDrive.turn(Math.toRadians(180));
            //drive_slow();
        } else {
            stopDrive();
        }
    }

    public void timed_drive(double y, double x, double rx, double speed, double time) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        // while (opModeIsActive() &&  runTime.time() > time) {
        while (runTime.time() < time) {
            drive(y, x, 0, speed);
        }
        stopDrive();
    }
}


