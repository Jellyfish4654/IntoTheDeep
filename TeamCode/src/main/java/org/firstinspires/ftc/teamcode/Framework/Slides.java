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
    double kP = 0.03, kI = 0.0061, kD = 0.0004;
    private PIDController slideController;
    private double kPLeft = 0.01, kILeft = 0, kDLeft = 0.0002, kFLeft = 0.01;
//	private double kPRight = 0.01, kIRight = 0, kDRight = 0.0002, kFRight = 0.01;
    public static double f = 0;
    public static double leftTarget = 300;
    public static double rightTarget = 300;
    public final double high_set_left = 1480;
    public final double high_set_right = 1480;
    public final double transfer_set_left =-129;
    public final double transfer_set_right = -129;
    public final double low_set_left = -1350;
    public final double low_set_right = -1350;
    private final double ticks_in_degree = 587.3/360;
    private int targetPosition;
    private double voltageCompensation;

    public Slides(DcMotorEx slideMotorLeft, DcMotorEx slideMotorRight, VoltageSensor sensor) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;

        this.voltageSensor = sensor;
        this.timer = new ElapsedTime();
    }

    public void setTargetPositions(double TargetPositionLeft, double TargetPositionRight) {
        leftTarget = TargetPositionLeft;
        rightTarget = TargetPositionRight;
        timer.reset();
    }

    public void setLow() {
        setTargetPositions(low_set_left, low_set_right);
    }

    public void setTransfer() {
        setTargetPositions(transfer_set_left, transfer_set_right);
    }

    public void setHigh() {
        setTargetPositions(high_set_left, high_set_right);
    }

    public void update() {
        double elapsedTime = timer.seconds();
        control(slideMotorLeft, leftTarget);
        control(slideMotorRight, rightTarget);
    }

    private void control(DcMotorEx motor, double target) {
        double powerSlide = calculateMotorPower(motor, target, slideController);
        motor.setPower(powerSlide);
    }


    private double calculateMotorPower(DcMotorEx motor, double targetPosition, PIDController slideController) {
        int position = motor.getCurrentPosition();
        return slideController.calculate(position, targetPosition);
    }
    public int getTargetPosition()
    {
        return targetPosition;
    }
    public int getCurrentLeftPosition()
    {
        return slideMotorLeft.getCurrentPosition();
    }
    public int getCurrentRightPosition()
    {
        return slideMotorRight.getCurrentPosition();
    }

}