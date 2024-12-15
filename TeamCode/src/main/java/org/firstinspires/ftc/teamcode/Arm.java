package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;




public class Arm {
    private Robot robot;
    //    static private double pos_whitepixel = 0.215;
    public double target;
    public boolean speed_control = false;
    private double max_speed = 0.01;
    private double threshold = 0.005;

    private final double P = 0.01;
    public Arm(Robot robot) {
        this.robot = robot;
        this.target = robot.servoArm.getPosition();
    }
    public void setPosStarting(boolean sc_on){
        if (sc_on) {
            setSCTarget(robot.arm_pos_starting);
        } else {
            robot.servoArm.setPosition(robot.arm_pos_starting);
        }
    }
    public void setPosSample(boolean sc_on) {
        if (sc_on) {
            setSCTarget(robot.arm_pos_sample);
        } else {
            robot.servoArm.setPosition(robot.arm_pos_sample);
        }
    }
    public void setPosSampleTwo(boolean sc_on) {
        if (sc_on) {
            setSCTarget(robot.arm_pos_sample_two);
        } else {
            robot.servoArm.setPosition(robot.arm_pos_sample_two);
        }
    }
    public void setPosChamber(boolean sc_on) {
        if (sc_on) {
            setSCTarget(robot.arm_pos_chamber);
        } else {
            robot.servoArm.setPosition(robot.arm_pos_chamber);
        }
    }
    public void setPosSpecimen(boolean sc_on) {
        if (sc_on) {
            setSCTarget(robot.arm_pos_specimen);
        } else {
            robot.servoArm.setPosition(robot.arm_pos_specimen);
        }
    }
    public void setPosBasket(boolean sc_on){
        if (sc_on) {
            setSCTarget(robot.arm_pos_basket);
        } else {
            robot.servoArm.setPosition(robot.arm_pos_basket);
        }
    }
    public void setPosAbsolute(double pos) {
        robot.servoArm.setPosition(pos);
    }

    public void setSCTarget(double target) {
        speed_control = true;
        this.target = target;
    }
    public void setChamberPush() {
        robot.servoArm.setPosition(robot.arm_pos_autonomous_chamber);
    }

    public void operate() {
        if (speed_control) {
            double curr_pos = robot.servoArm.getPosition();
            double diff = target - curr_pos;
            if (Math.abs(diff) > threshold) {
                double next_speed = Math.max(Math.min(diff * P, max_speed), -max_speed);
                double next_pos = curr_pos + next_speed;
                robot.servoArm.setPosition(next_pos);
            } else {
                speed_control = false;
            }
        }
        robot.telemetry.addData("Arm Curr Pos:", robot.servoArm.getPosition());
        robot.telemetry.addData("Arm Target:", this.target);
        robot.telemetry.addData("Arm Speed Control: ", speed_control);
        //robot.telemetry.update();
    }
}



