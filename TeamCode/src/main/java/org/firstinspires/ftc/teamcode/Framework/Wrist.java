package org.firstinspires.ftc.teamcode.Framework;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final Servo servo;
    private final double taka = 0.5;
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



}
