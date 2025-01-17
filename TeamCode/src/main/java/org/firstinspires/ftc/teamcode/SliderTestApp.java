package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "SliderTestApp")
public class SliderTestApp extends LinearOpMode {
    public Robot robot;
    public Slider slider;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;

    RevBlinkinLedDriver.BlinkinPattern pattern;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        slider = new Slider(robot);
        arm = new Arm(robot);
        wrist = new Wrist(robot);
        claw = new Claw(robot);

        arm.setPosStarting(false);
        wrist.setPosStarting(false);
        waitForStart();
        while(opModeIsActive()) {
//          slider_operate();
            slider_joystick();
            arm_wrist_operate();
//           claw_operate();
        }
    }

    private void arm_wrist_operate()
    {
        if (gamepad2.a) {
            arm.setPosSample(true);
            wrist.setPosSample(true);
        } else if (gamepad2.x) {
            arm.setPosBasket(true);
            wrist.setPosBasket(true);
        } else if(gamepad2.y) {
            arm.setPosChamber(true);
            wrist.setPosHighChamber(true);
        } else if(gamepad2.b) {
            arm.setPosSpecimen(true);
            wrist.setPosSpecimen(true);
        }
    }


    public void slider_joystick() {
        if (gamepad2.left_stick_y != 0) {
            robot.motorSlider.setPower(-gamepad2.left_stick_y);
        } else {
            robot.motorSlider.setPower(0);
        }
        double pos = robot.motorSlider.getCurrentPosition();
        robot.telemetry.addData("Slider Current Position:", pos);
        robot.telemetry.update();
    }
        private void slider_pos() {
        if (gamepad2.dpad_up) {
            slider.LowBasket();
        } else if (gamepad2.dpad_down) {
            slider.LowChamber();
        } else if (gamepad2.dpad_left) {
            slider.HighBasket();
        } else if (gamepad2.dpad_right) {
            slider.HighChamber();
        }

    }

    private void claw_operate() {
        if (gamepad2.left_trigger > 0.9) {
            claw.open();
        } else if (gamepad2.right_trigger > 0.9) {
            claw.open();
        } else {
            claw.close();
        }
    }
}

