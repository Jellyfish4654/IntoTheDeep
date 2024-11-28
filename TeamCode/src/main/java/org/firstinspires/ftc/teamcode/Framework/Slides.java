package org.firstinspires.ftc.teamcode.Framework;

import androidx.annotation.NonNull;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slides {
    private DcMotorEx slideMotorLeft, slideMotorRight;
    private VoltageSensor voltageSensor;
    private ElapsedTime timer;
    private double kPLeft = 0.01, kILeft = 0, kDLeft = 0.0002;
    private double kPRight = 0.01, kIRight = 0, kDRight = 0.0002;
    //	private double kPLeft = 0.01, kILeft = 0, kDLeft = 0.0002, kFLeft = 0.01;
//	private double kPRight = 0.01, kIRight = 0, kDRight = 0.0002, kFRight = 0.01;
    private double lastLeftError = 0, lastRightError = 0;
    private double leftIntegralSum = 0, rightIntegralSum = 0;
    private double kG = 1;
    private final double ticks_in_degrees = 537.7 / 360.0;
    private int targetPosition;
    private double voltageCompensation;

    public Slides(DcMotorEx slideMotorLeft, DcMotorEx slideMotorRight, VoltageSensor sensor) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;

        this.voltageSensor = sensor;
        this.timer = new ElapsedTime();
        setTargetPosition(0);
    }

    public void setTargetPosition(int TargetPosition) {
        targetPosition=TargetPosition;
        timer.reset();
    }

    public void update() {
        double elapsedTime = timer.seconds();

        leftControl();
        rightControl();
    }

    private void leftControl() {
        double leftPower = calculateMotorPowerLeft(slideMotorLeft, targetPosition);
        slideMotorLeft.setPower(leftPower);
    }

    private void rightControl() {
        double rightPower = calculateMotorPowerRight(slideMotorRight, targetPosition);
        slideMotorRight.setPower(rightPower);
    }

    private double calculateMotorPowerLeft(DcMotorEx motor, int targetPosition) {
        timer.reset();
        int currentPosition = motor.getCurrentPosition();
        int error = targetPosition - currentPosition;
        double derivative = (error - lastLeftError) / timer.seconds();
        double integralSum = leftIntegralSum + (error * timer.seconds());
        double out = (kPLeft * error) + (kILeft * integralSum) + (kDLeft * derivative) * kG;
        leftIntegralSum = integralSum;
        lastLeftError = error;
        return out;
    }
    private double calculateMotorPowerRight(DcMotorEx motor, int targetPosition) {
        timer.reset();
        int currentPosition = motor.getCurrentPosition();
        int error = targetPosition - currentPosition;
        double derivative = (error - lastRightError) / timer.seconds();
        double integralSum = rightIntegralSum + (error * timer.seconds());
        double out = (kPRight * error) + (kIRight * integralSum) + (kDRight * derivative) * kG;
        lastRightError = error;
        rightIntegralSum = integralSum;
        return out;
    }
    public int getTargetPosition()
    {
        return targetPosition;
    }

}