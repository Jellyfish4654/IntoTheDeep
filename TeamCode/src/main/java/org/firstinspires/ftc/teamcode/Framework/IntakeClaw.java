package org.firstinspires.ftc.teamcode.Framework;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClaw {
    private Servo clawServo;

    public IntakeClaw(Servo servo){
        this.clawServo = servo;
        /* might have to change the "clawServo" in the above line to something else,
        not sure if it needs to match something in the hardware */
    }

    public void setClawPos(double stickVal){
        double position = Math.abs(stickVal);
        clawServo.setPosition(position);
    }

    public void openClaw(){
        clawServo.setPosition(0.79);
        /* inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is open */

    }

    public void closeClaw(){
        clawServo.setPosition(0.5);
        /*
        inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is closed */
    }
    public void clawToggle() {
        if (clawServo.getPosition() == 0.5) {
            openClaw();
        } else {
            closeClaw();
        }
    }

    public double getClawPosition(){
//        telemetry.addData("claw position", hi);
        return clawServo.getPosition();
    }
    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(0.79);
            return false;
        }
    }
    public Action clawOpen () {
        return new IntakeClaw.ClawOpen();
    }
    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(0.5);;
            return false;
        }
    }
    public Action clawClose() {
        return new IntakeClaw.ClawClose();
    }
}
