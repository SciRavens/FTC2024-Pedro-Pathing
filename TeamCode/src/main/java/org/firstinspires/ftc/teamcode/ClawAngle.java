package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawAngle {
    Servo servo;
    private boolean horizontal = true;
    double horizontal_pos;
    double vertical_pos;


    public ClawAngle(Robot robot) {
        this.horizontal_pos = robot.claw_horizontal;
        this.vertical_pos = robot.claw_vertical;
        this.servo = robot.servoCR;
        this.servo.setPosition(horizontal_pos);
        horizontal = true;
    }

    public void setHorizontal()
    {
        if(!horizontal) {
            servo.setPosition(horizontal_pos);
            horizontal = true;
        }
    }
    public void setVertical()
    {
        if (horizontal) {
            servo.setPosition(vertical_pos);
            horizontal = false;
        }
    }

    public void setPosAbsolute(double pos) {
        servo.setPosition(pos);
    }

}
