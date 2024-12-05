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
    private double kPLeft = 0.05, kILeft = 0, kDLeft = 0.0002;
    private double kPRight = 0.05, kIRight = 0, kDRight = 0.0002;
    //	private double kPLeft = 0.01, kILeft = 0, kDLeft = 0.0002, kFLeft = 0.01;
//	private double kPRight = 0.01, kIRight = 0, kDRight = 0.0002, kFRight = 0.01;
    private PIDController controller;
    public static double f = 0.1;
    public static int target = -177;
    private final double ticks_in_degree = 587.3/360;
    private int targetPosition;
    private double voltageCompensation;

    public Slides(DcMotorEx slideMotorLeft, DcMotorEx slideMotorRight, VoltageSensor sensor) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;

        this.voltageSensor = sensor;
        this.timer = new ElapsedTime();
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

    private double calculateMotorPowerLeft(DcMotorEx motor, double targetPosition) {
        timer.reset();
        controller = new PIDController(kPLeft, kILeft, kDLeft);
        int leftPos = slideMotorLeft.getCurrentPosition();
        double pid = controller.calculate(leftPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        double power = pid + ff;
        return power;
    }
    private double calculateMotorPowerRight(DcMotorEx motor, int targetPosition) {
        timer.reset();
        controller = new PIDController(kPRight, kIRight, kDRight);
        int rightPos = slideMotorLeft.getCurrentPosition();
        double pid = controller.calculate(rightPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        double power = pid + ff;
        return power;
    }
    public int getTargetPosition()
    {
        return targetPosition;
    }
    public int getCurrentLeftPosition()
    {
        return slideMotorLeft.getCurrentPosition();
    }

}