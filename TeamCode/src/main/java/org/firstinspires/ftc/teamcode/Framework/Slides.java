package org.firstinspires.ftc.teamcode.Framework;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.arcrobotics.ftclib.controller.PIDController;

public class Slides {
    private DcMotorEx slideMotorLeft, slideMotorRight;
    private PIDController leftController, rightController;
    private VoltageSensor voltageSensor;

    private double kPLeft = 0, kILeft = 0, kDLeft = 0;
    private double kPRight = 0, kIRight = 0, kDRight = 0;
    //values to be filled in during testing

    private double rightPIDOutput;
    private double leftPIDOutput;
    double voltageCompensation;
    public Slides(DcMotorEx slideMotorLeft, DcMotorEx slideMotorRight, VoltageSensor sensor) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;

        this.leftController = new PIDController(kPLeft, kILeft, kDLeft);
        this.voltageSensor = sensor;

    }

    public void setTargetPosition(int TargetPosition) {
        double startVelocity
    }
}
