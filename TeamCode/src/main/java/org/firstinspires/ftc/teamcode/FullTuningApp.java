package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "FullTuning")
public class FullTuningApp extends LinearOpMode {
    public Robot robot;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public Claw rightClaw;
    public Claw leftClaw;
    public double arm_start_pos = 0.0;
    public double arm_cur_pos = arm_start_pos;
    public double arm_inc = 0.05;
    public double wrist_start_pos = 0.0;
    public double wrist_cur_pos = wrist_start_pos;
    public double wrist_inc = 0.05;
    public double claw_angle_start_pos = 0.0;
    public double claw_angle_cur_pos = claw_angle_start_pos;
    public double claw_inc = 0.05;
    public double claw_start_pos = 0.0;
    public double claw_cur_pos = claw_start_pos;

    public boolean buttonPressed = false;
    public boolean arm_tuning = true;
    public boolean wrist_tuning = false;
    public boolean claw_angle_tuning = true;
    public boolean claw_tuning = false;
    public int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        arm = new Arm(robot);
        wrist = new Wrist(robot);
        rightClaw = new Claw(robot);
        leftClaw = new Claw(robot);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad2.dpad_left) {
                arm_tuning = true;
                wrist_tuning = false;
            } else if (gamepad2.dpad_right) {
                wrist_tuning = true;
                arm_tuning = false;
            }
            if (gamepad2.x) {
                claw_angle_tuning = true;
                claw_tuning = false;
            } else if (gamepad2.b) {
                claw_angle_tuning = false;
                claw_tuning = true;
            }
            if (arm_tuning) {
                arm_operate();
            }
            if (wrist_tuning) {
                wrist_operate();
            }
            if (claw_angle_tuning) {
                claw_angle_operate();
            }
            if (claw_tuning) {
                claw_operate();
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
    private void claw_angle_operate()
    {
        if (gamepad2.y && !buttonPressed) {
            if (claw_angle_cur_pos < 0.9) {
                claw_angle_cur_pos += wrist_inc;
            }
            buttonPressed = true;
        } else if (gamepad2.a && !buttonPressed) {
            if (claw_angle_cur_pos > 0.1) {
                claw_angle_cur_pos -= wrist_inc;
            }
            buttonPressed = true;
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }
        robot.servoCR.setPosition(claw_angle_cur_pos);
        robot.telemetry.addData("Claw Angle Current Value:", claw_angle_cur_pos);
        robot.telemetry.update();
    }
    private void claw_operate()
    {
        if (gamepad2.y && !buttonPressed) {
            if (claw_cur_pos < 0.9) {
                claw_cur_pos += wrist_inc;
            }
            buttonPressed = true;
        } else if (gamepad2.a && !buttonPressed) {
            if (claw_cur_pos > 0.1) {
                claw_cur_pos -= wrist_inc;
            }
            buttonPressed = true;
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }
        robot.servoCL.setPosition(claw_cur_pos);
        robot.telemetry.addData("Claw Current Value:", claw_cur_pos);
        robot.telemetry.update();
    }
}