package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    Leds leds;
    private int led_cur = 1;
    private double slider_pos;
    Path turn, turn2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        robot.follower.startTeleopDrive();
        slider = new Slider(robot);
        arm = new Arm(robot);
        wrist = new Wrist(robot);
        claw = new Claw(robot);
        clawAngle = new ClawAngle(robot);

        arm.setPosStarting(false);
        wrist.setPosStarting(false);
        clawAngle.setHorizontal();
        waitForStart();
        while(opModeIsActive()) {
            follower_operate();
            arm.operate();
            wrist.operate();
            slider_joystick();
            arm_wrist_operate();
            claw_operate();
            turning();
            robot.telemetry.update();
        }
    }


    private void follower_operate()
    {
        boolean strafeOnly = false;

        if (gamepad1.right_trigger > 0.5) {
            robot.follower.setMaxPower(0.25);
        } else if (gamepad1.left_trigger > 0.5) {
            // This will make the robot go only strafing...no forward or backward movement
                strafeOnly = true;
        } else {
            robot.follower.setMaxPower(1.0);
        }
        if (strafeOnly == true) {
            robot.follower.setTeleOpMovementVectors(0.0, -gamepad1.left_stick_x, 0.0);

        } else {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
        robot.follower.update();
    }


    private void turning() {
        if(gamepad1.right_bumper) {
            turn = new Path(new BezierLine(new Point(follower.getPose()), new Point(follower.getPose())));
            turn.setLinearHeadingInterpolation(follower.getPose().getHeading(), follower.getPose().getHeading() + 90);
            follower.followPath(turn);
        }
        if (gamepad1.left_bumper){
            turn2 = new Path(new BezierLine(new Point(follower.getPose()), new Point(follower.getPose())));
            turn2.setLinearHeadingInterpolation(follower.getPose().getHeading(), follower.getPose().getHeading() + 180);
            follower.followPath(turn2);
        }
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
            arm.setPosSample(true);
            wrist.setPosSample(true);

//        } else if (gamepad2.dpad_up) {
//            // TBD: fix this
//            arm.setPosChamber(false);
//            wrist.setPosHighChamber(false);
//            slider.HighChamber();
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
        } else if  (gamepad2.dpad_right) {
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
    private void leds_operate() {
        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            led_cur = (led_cur + 1) % leds.patterns.length;
            leds.setPattern(led_cur);
            telemetry.addData("SETTING COLOR", leds.patterns[led_cur].toString());
            telemetry.update();
        }
    }
}

