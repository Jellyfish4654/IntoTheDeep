package org.firstinspires.ftc.teamcode.Framework;

import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeClaw {
    private Servo clawServo;

    public OuttakeClaw(Servo servo){
        this.clawServo = servo;
        /* might have to change the "clawServo" in the above line to something else,
        not sure if it needs to match something in the hardware */
    }

    public void setClawPos(double stickVal){
        double position = Math.abs(stickVal);
        clawServo.setPosition(position);
    }

    public void openClaw(){
        clawServo.setPosition(0.45);
        /* inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is open */

    }

    public void closeClaw(){
        clawServo.setPosition(0.15);
        /*
        inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is closed */
    }
    public void clawToggle() {
        if (clawServo.getPosition() == 0.15) {
            openClaw();
        } else {
            closeClaw();
        }
    }

    public double getClawPosition(){
//        telemetry.addData("claw position", hi);
        return clawServo.getPosition();
    }
}
