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

    public IntakeRotatingArmServos(Servo servo1, Servo servo2)
    {
        this.armLeftServo = servo1;
        this.armRightServo = servo2;
        armLeftServo.setPosition(ARM_INTAKE);
        armRightServo.setPosition(ARM_DEPOSIT);
        armRightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void armIntakePosition()
    {
        armLeftServo.setPosition(ARM_INTAKE);
        armRightServo.setPosition(ARM_INTAKE);
    }
    public void armIntakeDeposit()
    {
        armLeftServo.setPosition(ARM_DEPOSIT);
        armRightServo.setPosition(ARM_DEPOSIT);
    }

    public void setOutput(int position)
    {
        armLeftServo.setPosition(position);
        armRightServo.setPosition(position);
    }

}
