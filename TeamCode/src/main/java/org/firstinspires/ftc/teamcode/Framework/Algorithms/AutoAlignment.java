package org.firstinspires.ftc.teamcode.Framework.Algorithms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AutoAlignment
{
    private final DcMotor[] motors;
    private final IMU imuSensor;
    private final PIDController pidController;
    private double targetAngle = 0;
    public double kP = 0.03;
    public double kI = 0.0;
    public double kD = 0.0004;
    public final double kToleranceDegrees = 2.0;

    // Constructor
    public AutoAlignment(DcMotor[] motors, IMU imuSensor)
    {
        this.motors = motors;
        this.imuSensor = imuSensor;
        this.pidController = new PIDController(kP, kI, kD);
    }

    public void setTargetAngle(double targetAngle)
    {
        this.targetAngle = targetAngle;
    }

    public void update() {
        YawPitchRollAngles orientation = imuSensor.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);
        currentYaw = normalizeAngle(currentYaw);
        double angleDifference = normalizeAngle(targetAngle - currentYaw);
        double correction = pidController.calculate(0, angleDifference);
        if (Math.abs(angleDifference) > kToleranceDegrees) {
            MotorPowers(correction);
        } else {
            MotorPowers(0);
        }
    }

    private void MotorPowers(double correction)
    {
        double rightPower = correction;
        double leftPower = -correction;

        motors[0].setPower(rightPower);
        motors[1].setPower(rightPower);
        motors[2].setPower(leftPower);
        motors[3].setPower(leftPower);
    }

    private double normalizeAngle(double angle)
    {
        angle = angle % 360;
        if (angle <= -180)
        {
            angle += 360;
        }
        else if (angle > 180)
        {
            angle -= 360;
        }
        return angle;
    }
}