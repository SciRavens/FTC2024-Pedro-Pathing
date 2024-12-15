package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ArmWristTuning")
public class ArmWristTuningApp extends LinearOpMode {
    public Robot robot;
    public Arm arm;
    public Wrist wrist;
    public double arm_start_pos = 0.5;
    public double arm_cur_pos = arm_start_pos;
    public double arm_inc = 0.01;
    public double wrist_start_pos = 0.5;
    public double wrist_cur_pos = wrist_start_pos;
    public double wrist_inc = 0.01;
    public boolean buttonPressed = false;
    public boolean arm_tuning = true;
    public boolean wrist_tuning = false;
    public int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        arm = new Arm(robot);
        wrist = new Wrist(robot);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad2.dpad_left) {
                arm_tuning = true;
                wrist_tuning = false;
            } else if (gamepad2.dpad_right) {
                wrist_tuning = true;
                arm_tuning = false;
            }
            if (arm_tuning) {
                arm_operate();
            }
            if (wrist_tuning) {
                wrist_operate();
            }
        }
    }
    private void arm_operate()
    {
        if (gamepad2.dpad_up && !buttonPressed) {
            if (arm_cur_pos < 0.9) {
                arm_cur_pos += arm_inc;
            }
            buttonPressed = true;
        } else if (gamepad2.dpad_down && !buttonPressed) {
            if (arm_cur_pos > 0.1) {
                arm_cur_pos -= arm_inc;
            }
            buttonPressed = true;
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }

        arm.setPosAbsolute(arm_cur_pos);
        //if (arm_cur_pos < 0.9)
         //   Log.e("FTC: ", "Inc:" + arm_inc + " Butt:" + buttonPressed + "cur: " + arm_cur_pos);
        robot.telemetry.addData("Increment:", arm_inc);
        robot.telemetry.addData("Button Pressed:", buttonPressed);
        robot.telemetry.addData("Arm Current Value:", arm_cur_pos);
        robot.telemetry.update();
    }
    private void wrist_operate()
    {
        if (gamepad2.dpad_up && !buttonPressed) {
            if (wrist_cur_pos < 0.9) {
                wrist_cur_pos += wrist_inc;
            }
            buttonPressed = true;
        } else if (gamepad2.dpad_down && !buttonPressed) {
            if (wrist_cur_pos > 0.1) {
                wrist_cur_pos -= wrist_inc;
            }
            buttonPressed = true;
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }
        wrist.setPosAbsolute(wrist_cur_pos);
        robot.telemetry.addData("Wrist Current Value:", wrist_cur_pos);
        robot.telemetry.update();
    }
}

