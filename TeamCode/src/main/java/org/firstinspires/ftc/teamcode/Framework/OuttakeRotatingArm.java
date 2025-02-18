package org.firstinspires.ftc.teamcode.Framework;

import androidx.annotation.NonNull;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeRotatingArm {
    private static final double ARM_INTAKE = 0.96;
    private static final double ARM_DEPOSIT = 0.645;
    private static final double ARM_CHAMBER = 0.71;
    private static final double ARM_GRAB = 0.72;
    private static final double ARM_INIT = 0.3;
    //replace values after testing
    private final Servo armLeftServo;
    double positionL = ARM_INTAKE;


    public double getCurrentPosition() {
        return armLeftServo.getPosition();
    }
    public OuttakeRotatingArm(Servo servo1)
    {
        this.armLeftServo = servo1;
        armLeftServo.setPosition(positionL);
        armLeftServo.setDirection(Servo.Direction.REVERSE);
    }
    public void armOuttakeIntake()
    {
        armLeftServo.setDirection(Servo.Direction.REVERSE);
        positionL = ARM_INTAKE;
        armLeftServo.setPosition(positionL);
    }
    public void armOuttakeDeposit()
    {
        if (positionL == ARM_INTAKE) {
            armLeftServo.setDirection(Servo.Direction.FORWARD);
            positionL = ARM_DEPOSIT;
        } else {
            armLeftServo.setDirection(Servo.Direction.REVERSE);
            positionL = 1-ARM_DEPOSIT;
        }
        armLeftServo.setPosition(positionL);
    }
    public void armOuttakeGrab()
    {
        armLeftServo.setDirection(Servo.Direction.FORWARD);
        positionL = ARM_GRAB;
        armLeftServo.setPosition(positionL);
    }
    public void armOuttakeChamber()
    {
        if (positionL == ARM_GRAB) {
            armLeftServo.setDirection(Servo.Direction.REVERSE);
            positionL = 1-ARM_CHAMBER;
        } else {
            armLeftServo.setDirection(Servo.Direction.FORWARD);
            positionL = ARM_CHAMBER;
        }
        armLeftServo.setPosition(positionL);
    }
    public void armOuttakeInit()
    {
        armLeftServo.setDirection(Servo.Direction.FORWARD);
        positionL = ARM_INIT;
        armLeftServo.setPosition(positionL);
    }

    public void setOutput()
    {
        armLeftServo.setPosition(positionL);
    }

    public class OuttakeDeposit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeftServo.setDirection(Servo.Direction.FORWARD);
            armLeftServo.setPosition(ARM_DEPOSIT);
            return armLeftServo.getPosition() != ARM_DEPOSIT;
        }
    }
    public Action outtakeDeposit() {
        return new OuttakeDeposit();
    }

    public class OuttakeGrabSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeftServo.setDirection(Servo.Direction.FORWARD);
            armLeftServo.setPosition(ARM_GRAB);
            return armLeftServo.getPosition() != ARM_GRAB;
        }
    }
    public Action outtakeGrabSpecimen() {
        return new OuttakeDeposit();
    }
    public class OuttakeTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeftServo.setDirection(Servo.Direction.REVERSE);
            armLeftServo.setPosition(ARM_INTAKE);
            return armLeftServo.getPosition() != ARM_INTAKE;
        }
    }
    public Action outtakeTransfer() {
        return new OuttakeTransfer();
    }
    public class OuttakeChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeftServo.setPosition(ARM_CHAMBER);
            return false;
        }
    }
    public Action outtakeChamber() {
        return new InstantAction(() ->
                armLeftServo.setPosition(ARM_CHAMBER));
    }


}
