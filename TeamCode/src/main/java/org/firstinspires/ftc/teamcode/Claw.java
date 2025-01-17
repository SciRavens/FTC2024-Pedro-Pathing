package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;




public class Claw {
    Servo servo;
    private boolean closed = true;
    double close_pos;
    double open_pos;
    double open_wide_pos;
    private double cur_pos = 0.0;


    public Claw(Robot robot) {
        this.close_pos = robot.claw_close;
        this.open_pos = robot.claw_open;
        this.open_wide_pos = robot.claw_open_wide;
        this.servo = robot.servoCL;
        this.servo.setPosition(close_pos);
        closed = true;
        cur_pos = close_pos;

    }

    public void open() {
        if (closed) {
            servo.setPosition(open_pos);
            cur_pos = open_pos;
            closed = false;
        }
    }

    public void open_wide() {
        if (closed) {
            servo.setPosition(open_wide_pos);
            cur_pos = open_wide_pos;
            closed = false;
        }
    }

    public void close() {
        if (!closed) {
            servo.setPosition(close_pos);
            cur_pos = close_pos;
            closed = true;
        }
    }

    public double getCurPos() {
        return cur_pos;
    }
    public void setPosAbsolute(double pos) {
        servo.setPosition(pos);
        cur_pos = pos;
    }


}
