package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "FullTuning")
public class FullTuningApp extends LinearOpMode {
    public Robot robot;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public ClawAngle clawAngle;
    public double arm_cur_pos = 0.0;
    public double arm_inc = 0.01;
    public double wrist_start_pos = 0.0;
    public double wrist_cur_pos = wrist_start_pos;
    public double wrist_inc = 0.01;
    public double claw_angle_start_pos = 0.0;
    public double claw_angle_cur_pos = claw_angle_start_pos;
    public double claw_inc = 0.05;
    public double claw_start_pos = 0.0;
    public double claw_cur_pos = claw_start_pos;

    public boolean buttonPressed = false;
    public boolean arm_tuning = false;
    public boolean wrist_tuning = false;
    public boolean claw_angle_tuning = false;
    public boolean claw_tuning = false;
    public int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        arm = new Arm(robot);
        wrist = new Wrist(robot);
        claw = new Claw(robot);
        clawAngle = new ClawAngle(robot);

        waitForStart();
        robot.telemetry.addData("ARM: x, WRIST: y, CLAW: a, CLAW_ANGLE: b", 0);
        while (opModeIsActive()) {
            if (gamepad2.x) {
                arm_tuning = true;
                wrist_tuning = false;
                claw_angle_tuning = false;
                claw_tuning = false;
                arm_cur_pos = arm.getCurPos();
            } else if (gamepad2.y) {
                arm_tuning = false;
                wrist_tuning = true;
                claw_angle_tuning = false;
                claw_tuning = false;
                wrist_cur_pos = wrist.getCurPos();
            }
            if (gamepad2.a) {
                arm_tuning = false;
                wrist_tuning = false;
                claw_angle_tuning = false;
                claw_tuning = true;
                claw_cur_pos = claw.getCurPos();
            } else if (gamepad2.b) {
                arm_tuning = false;
                wrist_tuning = false;
                claw_angle_tuning = true;
                claw_tuning = false;
                claw_angle_cur_pos = clawAngle.getCurPos();
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
            if (gamepad2.left_trigger > 0.9) {
                slider_operate();
            }
            robot.telemetry.update();
        }
    }


    private void slider_operate() {
        if (gamepad2.left_stick_y != 0) {
            robot.motorSlider.setPower(-gamepad2.left_stick_y);
        } else {
            robot.motorSlider.setPower(0);
        }
        double pos = robot.motorSlider.getCurrentPosition();
        robot.telemetry.addData("Slider Current Position:", pos);
    }
    private void arm_operate() {
        if (gamepad2.dpad_up && !buttonPressed) {
            if (arm_cur_pos < 0.99) {
                arm_cur_pos += arm_inc;
            }
            buttonPressed = true;
            arm.setPosAbsolute(arm_cur_pos);
        } else if (gamepad2.dpad_down && !buttonPressed) {
            if (arm_cur_pos > 0.01) {
                arm_cur_pos -= arm_inc;
            }
            buttonPressed = true;
            arm.setPosAbsolute(arm_cur_pos);
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }
        robot.telemetry.addData("Arm Current Value:", arm.getCurPos());
    }

    private void wrist_operate() {
        if (gamepad2.dpad_up && !buttonPressed) {
            if (wrist_cur_pos < 0.99) {
                wrist_cur_pos += wrist_inc;
            }
            buttonPressed = true;
            wrist.setPosAbsolute(wrist_cur_pos);
        } else if (gamepad2.dpad_down && !buttonPressed) {
            if (wrist_cur_pos > 0.01) {
                wrist_cur_pos -= wrist_inc;
            }
            buttonPressed = true;
            wrist.setPosAbsolute(wrist_cur_pos);
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }
        robot.telemetry.addData("Wrist Current Value:", wrist.getCurPos());

    }

    private void claw_angle_operate() {
        if (gamepad2.dpad_up && !buttonPressed) {
            if (claw_angle_cur_pos < 0.99) {
                claw_angle_cur_pos += wrist_inc;
            }
            buttonPressed = true;
            robot.servoCR.setPosition(claw_angle_cur_pos);
        } else if (gamepad2.dpad_down && !buttonPressed) {
            if (claw_angle_cur_pos > 0.01) {
                claw_angle_cur_pos -= wrist_inc;
            }
            buttonPressed = true;
            robot.servoCR.setPosition(claw_angle_cur_pos);
        } else {
            if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                buttonPressed = false;
        }
        robot.telemetry.addData("Claw Angle Current Value:", clawAngle.getCurPos());
    }

    private void claw_operate() {
        if (gamepad2.right_trigger > 0.9) {
            claw.open();
        } else if (gamepad2.left_trigger > 0.9) {
                claw.close();
        } else if (gamepad2.dpad_up && !buttonPressed) {
                if (claw_cur_pos < 0.99) {
                    claw_cur_pos += claw_inc;
                }
                buttonPressed = true;
                robot.servoCL.setPosition(claw_cur_pos);
        } else if (gamepad2.dpad_down && !buttonPressed) {
                if (claw_cur_pos > 0.01) {
                    claw_cur_pos -= claw_inc;
                }
                buttonPressed = true;
                robot.servoCL.setPosition(claw_cur_pos);
        } else {
                if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                    buttonPressed = false;
        }
        robot.telemetry.addData("Claw Current Value:", claw.getCurPos());
    }
}