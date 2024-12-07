package org.firstinspires.ftc.teamcode.Framework;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRotatingArmServos {
    private static final double ARM_INTAKE = 0.5;
    private static final double ARM_DEPOSIT = 1;
    //replace values after testing
    private final Servo armLeftServo;
    private final Servo armRightServo;
    double positionL = ARM_INTAKE;
    double positionR = ARM_INTAKE;

    public IntakeRotatingArmServos(Servo servo1, Servo servo2)
    {
        this.armLeftServo = servo1;
        this.armRightServo = servo2;
        armLeftServo.setPosition(positionL);
        armRightServo.setPosition(positionR);
        armRightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void armOuttakeIntake()
    {
        positionL = ARM_INTAKE;
        positionR = ARM_INTAKE;
        armLeftServo.setPosition(positionL);
        armRightServo.setPosition(positionR);
    }
    public void armIntakeDeposit()
    {
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

}
