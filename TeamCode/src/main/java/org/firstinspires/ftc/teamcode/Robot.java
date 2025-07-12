package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Robot {
    public DcMotor rightFront = null; // Front Right
    public DcMotor leftFront = null; // Front Left
    public DcMotor rightRear = null; // Back Right
    public DcMotor leftRear = null; // Back Left

    public DcMotorEx motorSlider; // Slider
    public Servo servoArmLeft; // Elbow or Arm
    public Servo servoArmRight; // Elbow or Arm

    //public CRServo servoArm; // Elbow or Arm
    public Servo servoWrist; // Wrist

    public Servo servoCR; // Claw Right
    public Servo servoCL; // Claw left
    public Telemetry telemetry;

    public WebcamName webcam;
    public Follower follower;

    // Claw positions
    public double claw_open = 0;
    public double claw_close = 0.6;

    //ClawAngle positions
    public double claw_horizontal = 0.38;
    public double claw_vertical = 0.7;

    // Arm positions
    public double arm_pos_starting = 0.98;
    public double arm_pos_fold = arm_pos_starting;
    public  double arm_pos_sample = 0.4;
    public double arm_pos_sample_two = 0.43;
    public double arm_pos_basket = 0.75; //0.55
    public double arm_pos_specimen = 0.38;
    public double arm_pos_autonomous_chamber = 0.225;//0.425
    public double arm_pos_chamber = 0.48; //0.65

    // Wrist positions
    public double wrist_pos_starting = 0.66;
    public double wrist_pos_fold = 0.9;
    public double wrist_pos_sample  = 0.69;//0.64
    public double wrist_pos_sample_two = 0.19;
    public double wrist_pos_specimen = 0.35;
    public double wrist_pos_high_chamber = 0.25; //0.5
    public double wrist_pos_autonomous_chamber = 0.15;
    public double wrist_pos_basket = 0.72;

    // Slider positions
    public int slider_Initial_Pose_ticks = 0;
    public int slider_LowBasket_ticks = 2050;
    public int slider_HighBasket_ticks = 2100; // finished needs testing
    public int slider_LowChamber_ticks = 300;
    public int slider_HighChamber_ticks = 675; // 675 finished needs testing

    public int slider_ChamberAuton_ticks = 10;

    public RevBlinkinLedDriver led;
    public int wrist_pos_chamber_auton;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorSlider = hardwareMap.get(DcMotorEx.class, "sliders");
        servoArmLeft = hardwareMap.get(Servo.class, "left_arm");
        servoArmRight = hardwareMap.get(Servo.class, "right_arm");
        servoWrist = hardwareMap.get(Servo.class, "claw_arm");
        servoCL = hardwareMap.get(Servo.class, "claw_left");
        servoCR = hardwareMap.get(Servo.class, "claw_right");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        follower = new Follower(hardwareMap);
    }

}
