package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.Gamepad;

public class Wrist {
    private Robot robot;
    public double target;
    private double speed = 0.005;

    public boolean speed_control = false;

    public Wrist(Robot robot) {

        this.robot = robot;
        this.target = robot.servoWrist.getPosition();
    }

    public void setPosStarting(boolean sc_on){
        if (sc_on) {
            setSCTarget(robot.wrist_pos_starting);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_starting);
        }
    }
    public void setPosSample(boolean sc_on)
    {
        if (sc_on) {
            setSCTarget(robot.wrist_pos_sample);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_sample);
        }
    }
    public void setPosSampleTwo(boolean sc_on)
    {
        if (sc_on) {
            setSCTarget(robot.wrist_pos_sample_two);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_sample_two);
        }
    }
    public void setPosSpecimen(boolean sc_on)
    {
        if (sc_on) {
            setSCTarget(robot.wrist_pos_specimen);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_specimen);
        }
    }
    public void setPosHighChamber(boolean sc_on) {
        if (sc_on) {
            setSCTarget(robot.wrist_pos_high_chamber);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_high_chamber);
        }
    }
   // public void setPosLowChamber() {robot.servoWrist.setPosition(robot.wrist_pos_low_chamber);}

    public void setPosBasket(boolean sc_on)
    {
        if (sc_on) {
            setSCTarget(robot.wrist_pos_basket);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_basket);
        }
    }

    public void setPosChamberAuton(boolean sc_on)
    {
        if (sc_on) {
            setSCTarget(robot.wrist_pos_chamber_auton);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_chamber_auton);
        }
    }

    public void setChamberWristPush(boolean sc_on) {
        if (sc_on) {
            setSCTarget(robot.wrist_pos_autonomous_chamber);
        } else {
            robot.servoWrist.setPosition(robot.wrist_pos_autonomous_chamber);
        }
    }

    public void setPosAbsolute(double pos)
    {
        robot.servoWrist.setPosition(pos);
    }

    public void setSCTarget(double target) {
        speed_control = true;
        this.target = target;
    }

    public void operate() {
        if (speed_control) {
            double curr_pos = robot.servoWrist.getPosition();
            if (Math.abs(target - curr_pos) > speed) {
                double next_pos = curr_pos + speed * ((target > curr_pos) ? 1 : -1);
                robot.servoWrist.setPosition(next_pos);
            } else {
                speed_control = false;
            }
        }
        robot.telemetry.addData("Wrist Curr Pos:", robot.servoWrist.getPosition());
        robot.telemetry.addData("Wrist Target:", this.target);
        robot.telemetry.addData("Wrist Speed Control: ", speed_control);
    }

}
