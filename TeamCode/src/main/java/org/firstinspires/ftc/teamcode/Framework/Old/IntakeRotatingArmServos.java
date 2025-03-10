package org.firstinspires.ftc.teamcode.Framework.Old;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRotatingArmServos {
    private static final double ARM_INTAKE = 1;
    private static final double ARM_DEPOSIT = 0.6;
    //replace values after testing
    private final Servo armLeftServo;
    private final Servo armRightServo;

    public IntakeRotatingArmServos(Servo servo1, Servo servo2)
    {
        this.armLeftServo = servo1;
        this.armRightServo = servo2;
        armLeftServo.setPosition(ARM_INTAKE);
        armRightServo.setPosition(ARM_INTAKE);
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

    public void setOutput (double position)
    {
        armLeftServo.setPosition(position);
        armRightServo.setPosition(position);
    }

    public double getPositionLeft() {
        return armLeftServo.getPosition();
    }
    public double getPositionRight() {
        return armRightServo.getPosition();
    }

}
