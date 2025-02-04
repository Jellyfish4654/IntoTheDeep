package org.firstinspires.ftc.teamcode.Framework;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final Servo servo;
    private final double taka = 0.7;
    // these are going to change depending on what position the irl servo is set to... let's change later
    private final double matt = 0;
    private int position;

    public Wrist(Servo servo){
        this.servo = servo;
    }

    public void setPosUp() {
        changePosition(taka);
    }

    public void setPosDown() {
        changePosition(matt);
    }

    private void changePosition(double position) {
        servo.setPosition(position);
    }

    public double getCurrentPosition() {
        return position;
    }
    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(taka);
            return servo.getPosition() != taka;
        }
    }
    public Action wristUp() {
        return new WristUp();
    }
    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(matt);
            return servo.getPosition() != matt;
        }
    }
    public Action wristDown() {
        return new WristDown();
    }



}
