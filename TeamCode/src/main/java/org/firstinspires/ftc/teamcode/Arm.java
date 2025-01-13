package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;




public class Arm {
    private Robot robot;
    //    static private double pos_whitepixel = 0.215;
    private double target;
    private boolean speed_control = false;
    private double max_speed = 0.5; //0.1
    private double threshold = 0.005;
    private final double P = 0.03;

    private double cur_pos = 0.0;

    public Arm(Robot robot) {
        this.robot = robot;
        cur_pos = robot.servoArmLeft.getPosition();
        target = cur_pos;
    }

    private void setPosforBoth(double pos, boolean sc_on) {
        if (sc_on) {
            setSCTarget(pos);
        } else {
            robot.servoArmLeft.setPosition(pos);
            robot.servoArmRight.setPosition(pos);
            cur_pos = pos;
        }
    }
    public double getCurPos()
    {
        return cur_pos;
    }

    public void setPosStarting(boolean sc_on){
        setPosforBoth(robot.arm_pos_starting, sc_on);
    }
    public void setPosFold(boolean sc_on){
        setPosforBoth(robot.arm_pos_fold, sc_on);
    }
    public void setPosSample(boolean sc_on) {
        setPosforBoth(robot.arm_pos_sample, sc_on);
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
    public void setPosPark(boolean sc_on){
        setPosforBoth(robot.arm_pos_park, sc_on);
    }
    public void setPosChamberBack(boolean sc_on){
        setPosforBoth(robot.arm_pos_chamber_back, sc_on);
    }
    public void setPosAbsolute(double pos) {
        setPosforBoth(pos, false);
    }


    private void setSCTarget(double target) {
        speed_control = true;
        this.target = target;
    }

    /*
    public void setChamberPush() {
        setPosforBoth(robot.arm_pos_autonomous_chamber);
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
                setPosforBoth(target, false);
                speed_control = false;
            }
        }
        //robot.telemetry.addData("Arm Curr Pos:", robot.servoArmLeft.getPosition());
        //robot.telemetry.addData("Arm Target:", this.target);
        //robot.telemetry.addData("Arm Speed Control: ", speed_control);
    }
}



