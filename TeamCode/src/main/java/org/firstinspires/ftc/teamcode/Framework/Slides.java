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
    double kP = 0.06, kI = 0.0061, kD = 0.0002;
    private PIDController slideController;
    //	private double kPLeft = 0.01, kILeft = 0, kDLeft = 0.0002, kFLeft = 0.01;
//	private double kPRight = 0.01, kIRight = 0, kDRight = 0.0002, kFRight = 0.01;
    public static double f = 0;
    public static int target = 300;
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
        control(slideMotorLeft);
        control(slideMotorRight);
    }

    private void control(DcMotorEx motor) {
        double powerSlide = calculateMotorPower(motor, targetPosition, slideController);
        motor.setPower(powerSlide);
    }


    private double calculateMotorPower(DcMotorEx motor, double targetPosition, PIDController slideController) {
        slideController.setPID(kP, kI, kD);
        int position = motor.getCurrentPosition();
        return slideController.calculate(position, target);
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