package org.firstinspires.ftc.teamcode.Framework;
import org.firstinspires.ftc.teamcode.Framework.Profiles.MotionProfile;
import org.firstinspires.ftc.teamcode.Framework.Profiles.MotionState;
import org.firstinspires.ftc.teamcode.Framework.Profiles.MotionProfileGenerator;
import androidx.annotation.NonNull;

<<<<<<< HEAD
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
=======
import com.qualcomm.robotcore.hardware.DcMotor;
>>>>>>> df8e35307fb93ca061ef4c30201d535b0c402aa7
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake
{
    private DcMotorEx armMotor;
    private VoltageSensor voltageSensor;
    private PIDController controller;
    private MotionProfile profile;
    private ElapsedTime timer;
    private final double ticks_in_degrees = 537.7 / 360.0;
    private double maxVelocity = 2000; // True max is ~2200
    private double maxAcceleration = 29000; // True max is ~ 30000
    private double kP = 0.01, kI = 0, kD = 0.0002, kF1 = 0.008, kF2 = 0.015, kF3 = 0.04;
    ;
    private double PIDOutput;
    private int targetPosition;
    private double voltageCompensation;
    
    private boolean initialized = false;
    public Intake(DcMotorEx armMotor, VoltageSensor sensor) {
        this.armMotor = armMotor;

        this.controller = new PIDController(kP, kI, kD);

        this.voltageSensor = sensor;
        this.timer = new ElapsedTime();
        setTargetPosition(0);
    }

    public void setTargetPosition(int TargetPosition) {
        double startVelocity = 0;

        MotionState startState = new MotionState(armMotor.getCurrentPosition(), startVelocity);
        MotionState endState = new MotionState(TargetPosition, 0);
        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(startState, endState, maxVelocity, maxAcceleration);

        targetPosition=TargetPosition;
        timer.reset();
    }

    public void update() {
        double elapsedTime = timer.seconds();

        MotionState state = profile.get(elapsedTime);
        control(state);
    }

    private void control(MotionState targetState) {
        double leftPower = calculateMotorPower(armMotor, targetState, controller);
        PIDOutput=leftPower;
        armMotor.setPower(leftPower);
    }

<<<<<<< HEAD
    private double calculateMotorPower(DcMotorEx motor, MotionState targetState, PIDController controller) {
        int currentPosition = motor.getCurrentPosition();
        double power = controller.calculate(currentPosition, targetState.getX());
=======
        start() {
            if (Gamepad1.b.isPressed) {
                int desiredPosition = 1000
                armMotor.setTargetPosition (desiredPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
        
            if (!currentGamepad1.b && previousGamepad1.b) {
                armMotor.setPosition(armMotor.getPosition() - 0.1);
                }
>>>>>>> df8e35307fb93ca061ef4c30201d535b0c402aa7

        if (currentPosition > 900) {
            double angle = (targetState.getX() - currentPosition) / ticks_in_degrees;
            double ff = Math.cos(Math.toRadians(angle)) * kF1;
            power += ff;
        } else if (currentPosition > 300) {
            double angle = (targetState.getX() - currentPosition) / ticks_in_degrees;
            double ff = Math.cos(Math.toRadians(angle)) * kF2;
            power += ff;
        } else {
            double angle = (targetState.getX() - currentPosition) / ticks_in_degrees;
            double ff = Math.cos(Math.toRadians(angle)) * kF3;
            power += ff;
        }
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
        return targetPosition;
    }
}

