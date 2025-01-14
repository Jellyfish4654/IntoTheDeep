package org.firstinspires.ftc.teamcode.Framework;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClaw {
    private Servo clawServo;

    private static final double CLAW_OPEN = 0.4;
    private static final double CLAW_CLOSE = 0.7;
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
        clawServo.setPosition(CLAW_OPEN);
        /* inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is open */

    }

    public void closeClaw(){
        clawServo.setPosition(CLAW_CLOSE);
        /*
        inside the parentheses should be the
        number 0.0 --> 1.0 which is the position of the servo when claw is closed */
    }
    public void clawToggle() {
        if (clawServo.getPosition() == CLAW_CLOSE) {
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
            clawServo.setPosition(CLAW_OPEN);
            return clawServo.getPosition() != CLAW_OPEN;
        }
    }
    public Action clawOpen () {
        return new IntakeClaw.ClawOpen();
    }
    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(CLAW_CLOSE);
            return clawServo.getPosition() != CLAW_CLOSE;
        }
    }
    public Action clawClose() {
        return new IntakeClaw.ClawClose();
    }
}
