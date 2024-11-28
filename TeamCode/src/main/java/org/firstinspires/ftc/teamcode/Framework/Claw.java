package org.firstinspires.ftc.teamcode.Framework;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo clawServo;

    public void init(HardwareMap hwMap){
        clawServo = hwMap.get(Servo.class, "clawServo");
        /* might have to change the "clawServo" in the above line to something else,
        not sure if it needs to match something in the hardware */
    }

    public void openClaw(){
        /* clawServo.setPosition(?); inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is open */

    }

    public void closeClaw(){
        /* clawServo.setPosition(?); inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is closed */
    }

    public void getClawPosition(){
        telemetry.addData("claw position", clawServo.getPosition());
    }
}
