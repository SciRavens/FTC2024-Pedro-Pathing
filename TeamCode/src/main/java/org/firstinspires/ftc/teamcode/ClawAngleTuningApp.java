package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ClawAngleTuning")
public class ClawAngleTuningApp extends LinearOpMode {
    public Robot robot;
    public double start_pos = 0.5;
    public double cur_pos = start_pos;
    public double claw_inc = 0.05;
    public boolean buttonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            operate();
        }
    }


    private void operate() {
        if (gamepad2.dpad_up && !buttonPressed) {
            if (cur_pos <= 1.0) {
                cur_pos += claw_inc;
            }
            buttonPressed = true;
        } else if (gamepad2.dpad_down && !buttonPressed) {
            if (cur_pos >= 0) {
                cur_pos -= claw_inc;
            }
            buttonPressed = true;
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }
        robot.servoCR.setPosition(cur_pos);
        robot.telemetry.addData("Current Value:", cur_pos);
        robot.telemetry.update();
    }
}

