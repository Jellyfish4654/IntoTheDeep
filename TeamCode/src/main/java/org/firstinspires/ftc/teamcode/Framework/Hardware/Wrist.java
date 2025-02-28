package org.firstinspires.ftc.teamcode.Framework.Hardware;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final Servo servo;
    private final double up = 0.77;
    // these are going to change depending on what position the irl servo is set to... let's change later
    private final double down = 0.1875;
    private int position;

    public Wrist(Servo servo){
        this.servo = servo;
    }

    public void setPosUp() {
        changePosition(up);
    }

    public void setPosDown() {
        changePosition(down);
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
            servo.setPosition(up);
            return servo.getPosition() != up;
        }
    }
    public Action wristUp() {
        return new WristUp();
    }
    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(down);
            return servo.getPosition() != down;
        }
    }
    public Action wristDown() {
        return new WristDown();
    }



}
