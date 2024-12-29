package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.Gamepad;

public class Wrist {
    private Robot robot;
    private double target;
    private double speed = 0.005;

    private boolean speed_control = false;
    private double cur_pos = 0.0;

    public Wrist(Robot robot) {

        this.robot = robot;
        this.target = robot.servoWrist.getPosition();
        cur_pos = target;

    }

    private void setPos(double pos, boolean sc_on)
    {
        if (sc_on) {
            setSCTarget(pos);
        } else {
            robot.servoWrist.setPosition(pos);
            cur_pos = pos;
        }
    }

    public void setPosStarting(boolean sc_on){
        setPos(robot.wrist_pos_starting, sc_on);
    }
    public void setPosFold(boolean sc_on){
        setPos(robot.wrist_pos_fold, sc_on);
    }
    public void setPosSample(boolean sc_on)
    {
        setPos(robot.wrist_pos_sample, sc_on);
    }
    public void setPosSampleTwo(boolean sc_on)
    {
        setPos(robot.wrist_pos_sample_two, sc_on);
    }
    public void setPosSpecimen(boolean sc_on)
    {
        setPos(robot.wrist_pos_specimen, sc_on);
    }
    public void setPosHighChamber(boolean sc_on) {
        setPos(robot.wrist_pos_high_chamber, sc_on);
    }
   // public void setPosLowChamber() {robot.servoWrist.setPosition(robot.wrist_pos_low_chamber);}

    public void setPosBasket(boolean sc_on)
    {
        setPos(robot.wrist_pos_basket, sc_on);
    }

    public void setPosAbsolute(double pos)
    {
        robot.servoWrist.setPosition(pos);
    }

    private void setSCTarget(double target) {
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
    public double getCurPos() {
        return cur_pos;
    }
}
