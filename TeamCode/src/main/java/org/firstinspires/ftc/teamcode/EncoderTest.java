package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {
    public Robot robot;
    public DcMotorEx left, right, strafe;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder strafeEncoder;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

        leftEncoder.setDirection(Encoder.REVERSE);
        rightEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.FORWARD);

        waitForStart();
        while(opModeIsActive()) {
            get_ticks();
            if (gamepad1.x) {
                stopall();
                robot.leftFront.setPower(1.0);
            }
            if (gamepad1.y) {
                stopall();
                robot.leftRear.setPower(1.0);
            }
            if (gamepad1.a) {
                stopall();
                robot.rightFront.setPower(1.0);
            }
            if (gamepad1.b) {
                stopall();
                robot.rightRear.setPower(1.0);
            }
            robot.telemetry.update();
        }
    }

    public void stopall()
    {
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftRear.setPower(0);
        robot.leftFront.setPower(0);
    }


    public void get_ticks() {
        rightEncoder.update();
        leftEncoder.update();
        strafeEncoder.update();
        robot.telemetry.addData("rightEncoder", rightEncoder.getDeltaPosition());
        robot.telemetry.addData("LeftEncoder", leftEncoder.getDeltaPosition());
        robot.telemetry.addData("StrafeEncoder", strafeEncoder.getDeltaPosition());
        robot.telemetry.addData("Right ticks -- rightFront", robot.rightFront.getCurrentPosition());
        robot.telemetry.addData("Left ticks -- BackLeft", robot.leftRear.getCurrentPosition());
        robot.telemetry.addData("BackRight ticks -- rightRear", robot.rightRear.getCurrentPosition());
    }
}

