package org.firstinspires.ftc.teamcode.Framework;

import androidx.annotation.NonNull;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeRotatingArm {
    private DcMotorEx armMotor;
    private VoltageSensor voltageSensor;
    private ElapsedTime timer;
    private double kP = 0.01, kI = 0, kD = 0.0002;
    //	private double kPLeft = 0.01, kILeft = 0, kDLeft = 0.0002, kFLeft = 0.01;
//	private double kPRight = 0.01, kIRight = 0, kDRight = 0.0002, kFRight = 0.01;
    private double lastError = 0;
    private double integralSum = 0;
    private double kG = 1;
    private final double ticks_in_degrees = 537.7 / 360.0;
    private int targetPosition;
    private double voltageCompensation;

    public IntakeRotatingArm(DcMotorEx armMotor, VoltageSensor voltageSensor) {
        this.armMotor = armMotor;
        this.voltageSensor = voltageSensor;
        this.timer = new ElapsedTime();
        setTargetPosition(0);
    }

    public void setTargetPosition(int TargetPosition) {
        targetPosition=TargetPosition;
        timer.reset();
    }

    public void update() {
        double elapsedTime = timer.seconds();

        control(0.5);
    }


    public void control(double rightPower) {
        rightPower = calculateMotorPower(armMotor, targetPosition);
        armMotor.setPower(rightPower);
    }

    private double calculateMotorPower(DcMotorEx motor, int targetPosition) {
        timer.reset();
        int currentPosition = motor.getCurrentPosition();
        int error = targetPosition - currentPosition;
        double derivative = (error - lastError) / timer.seconds();
        double tempIntegralSum = integralSum + (error * timer.seconds());
        double out = (kP * error) + (kI * tempIntegralSum) + (kD * derivative) * kG;
        lastError = error;
        integralSum = tempIntegralSum;
        return out;
    }
    public int getTargetPosition()
    {
        return targetPosition;
    }

    public int getCurrentPosition(){
        return armMotor.getCurrentPosition();
    }

}