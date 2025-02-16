package org.firstinspires.ftc.teamcode.Framework;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Extendo {
    private DcMotorEx Extendo;
    private VoltageSensor voltageSensor;
    private ElapsedTime timer;
    public final double defaultp = 0.02, defaulti = 0, defaultd = 0;
    public static double p = 0.02, i = 0, d = 0;
    private PIDController controller;
    private double target;
    private final int retract = 140;
    private final int extend = 0;
    private final double ticks_in_degree = 587.3/360;
    private int targetPosition;
    private double voltageCompensation;

    public Extendo (DcMotorEx Extendo, VoltageSensor sensor) {
        this.Extendo = Extendo;
        controller = new PIDController(p, i, d);

        this.voltageSensor = sensor;
        this.timer = new ElapsedTime();
    }

    public void setTargetPosition (double TargetPosition) {
        target = TargetPosition;
        timer.reset();
    }

    public void setRetract() {
        setTargetPosition(retract);
    }
    public void setExtend() {
        setTargetPosition(extend);
    }
    public void update() {
        double elapsedTime = timer.seconds();
        control(Extendo, target, controller);
    }

    private void control(DcMotorEx motor, double target, PIDController controller) {
        double powerExtendo = calculateMotorPower(motor, target, controller);
        motor.setPower(powerExtendo);
    }
    private void controlNoPID(DcMotorEx motor, double power) {
        motor.setPower(power);
    }


    private double calculateMotorPower(DcMotorEx motor, double targetPosition, PIDController controller) {
        double position = motor.getCurrentPosition();
        return controller.calculate(position, targetPosition);
    }
    public void enablePID() {
        p = defaultp;
        i = defaulti;
        d = defaultd;
    }
    public void disablePID() {
        p = 0;
        i = 0;
        d = 0;
    }
    public int getTargetPosition()
    {
        return targetPosition;
    }
    public int getCurrentPosition()
    {
        return Extendo.getCurrentPosition();
    }
    public class ExtendoExtend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Extendo.setTargetPosition(extend);
            control(Extendo, target, controller);
            return false;
        }
    }
    public Action extendoExtend() {
        return new ExtendoExtend();
    }
    public class ExtendoRetract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Extendo.setTargetPosition(retract);
            control(Extendo, target, controller);
            if (Extendo.getCurrentPosition() == target) {
                return false;
            } else {
                return true;
            }
        }
    }
    public Action extendoRetract() {
        return new ExtendoRetract();
    }

}
