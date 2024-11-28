package org.firstinspires.ftc.teamcode.Framework;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class IntakeRotatingArm
{
    private DcMotorEx armMotor;
    private VoltageSensor voltageSensor;
    private ElapsedTime timer;
    private double kP = 0.01, kI = 0, kD = 0.0002;
    ;
    private double PIDOutput;
    private int targetAngle;
    private double voltageCompensation;
    
    private boolean initialized = false;
    public IntakeRotatingArm(DcMotorEx armMotor, VoltageSensor sensor) {
        this.armMotor = armMotor;

        this.voltageSensor = sensor;
        this.timer = new ElapsedTime();
        setTargetPosition(0);
    }

    public void setTargetPosition(int TargetPosition) {
        double startVelocity = 0;


        timer.reset();
    }

    public void update() {
        double elapsedTime = timer.seconds();


        control(state);
    }

    private void control() {
        double leftPower = calculateMotorPower(armMotor, targetAngle);
        PIDOutput=leftPower;
        armMotor.setPower(leftPower);
    }


    private double calculateMotorPower() {

        voltageCompensation = 13.2 / voltageSensor.getVoltage();
        power *= voltageCompensation;

        return power;
    }

    public double getPIDOutput()
    {
        return PIDOutput;
    }

    public int getTargetPosition()
    {

        return targetAngle;
    }
}

