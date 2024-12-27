package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;




public class Arm {
    private Robot robot;
    private double left_neutral, right_neutral;
    //    static private double pos_whitepixel = 0.215;
    public double target;
    public boolean speed_control = false;
    private double max_speed = 0.01;
    private double threshold = 0.005;

    private final double P = 0.01;
    public Arm(Robot robot) {
        this.robot = robot;
        this.left_neutral = robot.servoArmLeft.getPosition();
        this.right_neutral = robot.servoArmRight.getPosition();
        this.target = robot.servoArmLeft.getPosition();

    }

    private boolean is_valid_position(double pos) {
//        double right_pos = get_right_pos(pos);
//        if (right_pos < 0.0 || right_pos > 1.0)
//            return false;
        return true;
    }

    private void setPosforBoth(double pos, boolean sc_on) {
        if (sc_on) {
            setSCTarget(pos);
        } else {
            robot.servoArmLeft.setPosition(pos);
            robot.servoArmRight.setPosition(get_right_pos(pos));
            robot.telemetry.addData(" Left Neutral Value : ", left_neutral);
            robot.telemetry.addData("Right Neutral value  ", right_neutral);
            robot.telemetry.addData(" Current Left Neutral Value : ", robot.servoArmLeft.getPosition());
            robot.telemetry.addData("Current Right Neutral value  ", robot.servoArmRight.getPosition());
            robot.telemetry.addData("Left Pos: ", pos);
            robot.telemetry.addData("Right pos: ", get_right_pos(pos));
           // robot.telemetry.update();
        }
    }

    private double get_right_pos(double pos) {
        return (1.0 - pos);
    }
    public void setPosStarting(boolean sc_on){
        setPosforBoth(robot.arm_pos_starting, sc_on);
    }
    public void setPosFold(boolean sc_on){
        setPosforBoth(robot.arm_pos_fold, sc_on);
    }
    public void setPosSample(boolean sc_on) {
        setPosforBoth(robot.arm_pos_fold, sc_on);

        }
    public void setPosSampleTwo(boolean sc_on) {
        setPosforBoth(robot.arm_pos_sample_two, sc_on);
    }
    public void setPosChamber(boolean sc_on) {
        setPosforBoth(robot.arm_pos_chamber, sc_on);
    }
    public void setPosSpecimen(boolean sc_on) {
        setPosforBoth(robot.arm_pos_specimen, sc_on);
    }
    public void setPosBasket(boolean sc_on){
        setPosforBoth(robot.arm_pos_basket, sc_on);
        }
    public void setPosAbsolute(double pos) {
        setPosforBoth(pos, false);
    }

    public void setSCTarget(double target) {
        if (!is_valid_position(target)) {
            return;
        }
        speed_control = true;
        this.target = target;
    }
    /*
    public void setChamberPush() {
        robot.servoArmLeft.setPosition(1 - robot.arm_pos_autonomous_chamber);
        robot.servoArmRight.setPosition(robot.arm_pos_autonomous_chamber);

    }

     */
    public void operate() {
        if (speed_control) {
            double curr_pos = robot.servoArmLeft.getPosition();
            double diff = target - curr_pos;
            if (Math.abs(diff) > threshold) {
                double next_speed = Math.max(Math.min(diff * P, max_speed), -max_speed);
                double next_pos = curr_pos + next_speed;
                setPosforBoth(next_pos, false);
            } else {
                speed_control = false;
            }
        }
        //robot.telemetry.addData("Arm Curr Pos:", robot.servoArmLeft.getPosition());
        //robot.telemetry.addData("Arm Target:", this.target);
        //robot.telemetry.addData("Arm Speed Control: ", speed_control);
        //robot.telemetry.update();
    }
}



