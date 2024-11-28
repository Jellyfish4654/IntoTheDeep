package org.firstinspires.ftc.teamcode.Framework;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo clawServo;

    public Claw (Servo servo){
        this.clawServo = servo;
        /* might have to change the "clawServo" in the above line to something else,
        not sure if it needs to match something in the hardware */
    }

    public void openClaw(){
        clawServo.setPosition(0.2);
        /* inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is open */

    }

    public void closeClaw(){
        clawServo.setPosition(0.4);
        /*
        inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is closed */
    }

    public double getClawPosition(){

        telemetry.addData("claw position", clawServo.getPosition());
        return clawServo.getPosition();
    }
}
