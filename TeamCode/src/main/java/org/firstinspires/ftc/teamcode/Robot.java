package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.security.PublicKey;

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

    // Claw positions
    public static double claw_open = 0.5;
    public static double claw_open_wide = 0.4;
    public static double claw_close = 0.7;

    //ClawAngle positions
    public static double claw_horizontal = 0.71;
    public static double claw_vertical = 0.37;

    // Arm positions
    public static double arm_pos_starting = 0.73; //0.75
    public static double arm_pos_fold = arm_pos_starting;
    public  static double arm_pos_sample = 0.13;//0.39
    public static double arm_pos_sample_two = 0.3;
    public static double arm_pos_basket = 0.47; //0.39
    public static double arm_pos_specimen = 0.09; //0.01
    public static double arm_pos_autonomous_chamber = 0.225;//0.425
    public static double arm_pos_chamber = 0.21; //0.65
    public static double arm_pos_chamber_back = 0.64; //0.63
    public static double arm_pos_park = 0.38;
    public static double arm_back_human = 0.63;


    // Wrist positions
    public static double wrist_pos_starting = 0.05;
    public static double wrist_pos_fold = 0.16;// 0.43 -> 0.65
    public static double wrist_pos_sample  = 0.47;//0.49
    public static double wrist_pos_sample_two = 0.67;
    public static double wrist_back_human = 0.58;
    public static double wrist_pos_specimen = 0.77;//0.73
    public static double wrist_pos_high_chamber = 0.87; //0.08
    public static double wrist_pos_high_chamber_back = 0.75; //0.76
    public static double wrist_pos_autonomous_chamber = 0.15;
    public static double wrist_pos_basket = 0.4;//0.56
    public static double wrist_pos_park = 0.46;

    // Slider positions
    public static int slider_Initial_Pose_ticks = 0;
    public static int slider_LowBasket_ticks = 2050;
    public static int slider_HighBasket_ticks = 2100; // finished needs testing
    public static int slider_LowChamber_ticks = 1220; //1400
    public static int slider_HighChamber_ticks = 680; // 675 finished needs testing
    public static int slider_HighChamberBack_ticks = 2200; // 675 finished needs testing


    public static int slider_ChamberAuton_ticks = 10;

    public static int wrist_pos_chamber_auton;
    public TouchSensor limitSwitch;

    public VoltageSensor voltageSensor;

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
        motorSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        servoArmLeft = hardwareMap.get(Servo.class, "left_arm");
        servoArmRight = hardwareMap.get(Servo.class, "right_arm");
        servoWrist = hardwareMap.get(Servo.class, "claw_arm");
        servoCL = hardwareMap.get(Servo.class, "claw_left");
        servoCR = hardwareMap.get(Servo.class, "claw_right");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

}
