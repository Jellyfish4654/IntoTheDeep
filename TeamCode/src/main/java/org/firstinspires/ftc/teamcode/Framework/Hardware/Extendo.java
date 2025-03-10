package org.firstinspires.ftc.teamcode.Framework.Hardware;
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
    public static double p = 0.015, i = 0, d = 0;
    private PIDController controller;
    private double target;
    private double retract = -60;
    private double initial = 0;
    private double extend = -1500;
    private final double initialToExtend = -1500;
    private final double initialToRetract = -20;

    public Extendo (DcMotorEx Extendo, VoltageSensor sensor) {
        this.Extendo = Extendo;
        controller = new PIDController(p, i, d);

        this.voltageSensor = sensor;
        this.timer = new ElapsedTime();
    }
    public void establishPositions() {
        initial = Extendo.getCurrentPosition();
        retract = initial + initialToRetract;
        extend = initial+initialToExtend;
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
    public int getCurrentPosition()
    {
        return Extendo.getCurrentPosition();
    }
    public class ExtendoExtend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setExtend();
            control(Extendo, target, controller);

            double pos = getCurrentPosition();

            if (Math.abs(pos - target) > 50) {
                return true;
            } else {
                Extendo.setPower(0);
                return false;
            }
        }
    }
    public Action extendoExtend() {
        return new ExtendoExtend();
    }
    public class ExtendoRetract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setRetract();
            control(Extendo, target, controller);

            double pos = getCurrentPosition();

            if (Math.abs(pos - target) > 50) {
                return true;
            } else {
                Extendo.setPower(0);
                return false;
            }
        }
    }
    public Action extendoRetract() {
        return new ExtendoRetract();
    }

    public class ExtendoRetractFull implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            target = initial;


            double pos = getCurrentPosition();

            if (pos != target) {
                Extendo.setPower(1);
                return true;
            } else {
                Extendo.setPower(0);
                return false;
            }
        }
    }
    public Action extendoRetractFull() {
        return new ExtendoRetractFull();
    }

}
