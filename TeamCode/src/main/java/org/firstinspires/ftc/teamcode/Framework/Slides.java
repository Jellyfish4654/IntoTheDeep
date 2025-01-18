package org.firstinspires.ftc.teamcode.Framework;

import androidx.annotation.NonNull;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slides {
    boolean pidenable;
    private DcMotorEx slideMotorLeft, slideMotorRight;
    private VoltageSensor voltageSensor;
    private ElapsedTime timer;
    public final double defaultp = 0.03, defaulti = 0.0061, defaultd = 0.0004;
    public static double pleft = 0, ileft = 0, dleft = 0;
    public static double pright = 0.03, iright = 0.0061, dright = 0.0004;
    int targetPosition;
    private PIDController lcontroller;
    private PIDController rcontroller;

    public static double f = 0;
    public static double leftTarget = 0;
    public static double rightTarget = 0;
    public final double high_set_left = 0;
    public final double high_set_right = 0;
    public final double transfer_set_left = 0;
    public final double transfer_set_right = 0;
    public final double low_set_left = 0;
    public final double low_set_right = 0;
    private final double ticks_in_degree = 587.3/360;
    private double voltageCompensation;

    public Slides(DcMotorEx slideMotorLeft, DcMotorEx slideMotorRight, VoltageSensor sensor) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;
        lcontroller = new PIDController(pleft, ileft, dleft);
        rcontroller = new PIDController(pleft, ileft, dleft);

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

    public void update(boolean PID, double joyStickValue) {
        double elapsedTime = timer.seconds();
        if (PID) {
            control(slideMotorLeft, leftTarget, lcontroller);
            control(slideMotorRight, rightTarget, rcontroller);
        } else {
            controlNoPID(slideMotorLeft, joyStickValue);
            controlNoPID(slideMotorRight, joyStickValue*0.7);
        }
    }

    private void control(DcMotorEx motor, double target, PIDController slideController) {
        double powerSlide;
        powerSlide = calculateMotorPowerWithPID(motor, target, slideController);
        motor.setPower(powerSlide);
    }
    private void controlNoPID(DcMotorEx motor, double power) {
        motor.setPower(power);
    }


    private double calculateMotorPowerWithPID(DcMotorEx motor, double targetPosition, PIDController slideController) {
        double position = motor.getCurrentPosition();
        return slideController.calculate(position, targetPosition);
    }
    public void enablePID() {
        pidenable = true;
        pleft = defaultp;
        ileft = defaulti;
        dleft = defaultd;
    }
    public void disablePID() {
        pidenable = false;
        pleft = 0;
        ileft = 0;
        dleft = 0;
    }
    public void togglePID() {
        if (pidenable) {
            pidenable = false;
            disablePID();
        } else if (!pidenable) {
            pidenable = true;
            enablePID();
        }
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

    public class SlidesUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                slideMotorLeft.setPower(0.8);
                slideMotorRight.setPower(0.8);
                initialized = true;
            }
            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);


            if (posLeft < 3000 && posRight < 3000) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
                return false;
            }

        }
    }
    public Action slidesUp() {
        return new SlidesUp();
    }
    public class SlidesDown implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                slideMotorLeft.setPower(-0.8);
                slideMotorRight.setPower(-0.8);
                initialized = true;
            }
            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);


            if (posLeft > 100 && posRight > 100) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
                return false;
            }

        }
    }
    public Action slidesDown() {
        return new SlidesDown();
    }

}