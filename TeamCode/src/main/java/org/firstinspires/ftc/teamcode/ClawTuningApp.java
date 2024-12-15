package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ClawTuning")
public class ClawTuningApp extends LinearOpMode {
    public Robot robot;
    public Claw claw;
    public double start_pos = 0.5;
    public double cur_pos = start_pos;
    public double claw_inc = 0.05;
    public boolean buttonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        claw = new Claw(robot);
        waitForStart();
        while (opModeIsActive()) {
            claw_operate();
        }
    }


    private void claw_operate() {
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
        robot.servoCL.setPosition(cur_pos);
        robot.telemetry.addData("Current Value:", cur_pos);
        robot.telemetry.update();
    }
}

