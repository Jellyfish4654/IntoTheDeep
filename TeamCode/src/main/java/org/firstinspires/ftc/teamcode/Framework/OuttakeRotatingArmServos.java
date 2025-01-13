package org.firstinspires.ftc.teamcode.Framework;

import androidx.annotation.NonNull;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeRotatingArmServos {
    private static final double ARM_INTAKE = 0.4;
    private static final double ARM_DEPOSIT = 1;
    //replace values after testing
    private final Servo armLeftServo;
    private final Servo armRightServo;
    double positionL = ARM_INTAKE;
    double positionR = ARM_INTAKE;

    public OuttakeRotatingArmServos(Servo servo1, Servo servo2)
    {
        this.armLeftServo = servo1;
        this.armRightServo = servo2;
        armLeftServo.setPosition(positionL);
        armRightServo.setPosition(positionR);
        armRightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void armOuttakeIntake()
    {
        armLeftServo.setDirection(Servo.Direction.REVERSE);
        armRightServo.setDirection(Servo.Direction.FORWARD);
        positionL = ARM_INTAKE;
        positionR = ARM_INTAKE;
        armLeftServo.setPosition(positionL);
        armRightServo.setPosition(positionR);
    }
    public void armOuttakeDeposit()
    {
        armLeftServo.setDirection(Servo.Direction.FORWARD);
        armRightServo.setDirection(Servo.Direction.REVERSE);
        positionL = ARM_DEPOSIT;
        positionR = ARM_DEPOSIT;
        armLeftServo.setPosition(positionL);
        armRightServo.setPosition(positionR);
    }

    public void setOutput()
    {
        armLeftServo.setPosition(positionL);
        armRightServo.setPosition(positionR);
    }

    public class OuttakeDeposit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeftServo.setDirection(Servo.Direction.FORWARD);
            armRightServo.setDirection(Servo.Direction.REVERSE);
            armLeftServo.setPosition(0.7);
            armRightServo.setPosition(0.7);
            return false;
        }
    }
    public Action outtakeDeposit() {
        return new OuttakeDeposit();
    }
    public class OuttakeTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeftServo.setDirection(Servo.Direction.REVERSE);
            armRightServo.setDirection(Servo.Direction.FORWARD);
            armLeftServo.setPosition(0.4);
            armRightServo.setPosition(0.4);
            double leftPos = armLeftServo.getPosition();
            double rightPos = armLeftServo.getPosition();
            if (leftPos == 0.4 && rightPos == 0.4) {
                return false;
            } else {
                return true;
            }
        }
    }
    public Action outtakeTransfer() {
        return new OuttakeTransfer();
    }

}
