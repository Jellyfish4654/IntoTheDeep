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
    double positionL = ARM_INTAKE;

    public OuttakeRotatingArmServos(Servo servo1)
    {
        this.armLeftServo = servo1;
        armLeftServo.setPosition(positionL);
    }

    public void armOuttakeIntake()
    {
        armLeftServo.setDirection(Servo.Direction.REVERSE);
        positionL = ARM_INTAKE;
        armLeftServo.setPosition(positionL);
    }
    public void armOuttakeDeposit()
    {
        armLeftServo.setDirection(Servo.Direction.FORWARD);
        positionL = ARM_DEPOSIT;
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

}
