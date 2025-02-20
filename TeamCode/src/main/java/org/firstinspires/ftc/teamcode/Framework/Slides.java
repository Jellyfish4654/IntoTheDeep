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
    public double offset_tracker;
    public final double defaultp = 0.01, defaulti = 0, defaultd = 0;
    public static double pleft = 0.005, ileft = 0, dleft = 0;
    public static double pright = 0.03, iright = 0.0061, dright = 0.0004;
    int targetPosition;
    private PIDController lcontroller;
    private PIDController rcontroller;

    public static double f = 0;
    public static double leftTarget = 0;
    public static double rightTarget = 0;
    public double highest_set_left = 9300;
    public final double high_to_highest = 2240;
    public final double highest_set_right = 0;
    public double high_set_left = 7186;
    public final double transfer_to_high = 1409;
    public final double high_set_right = 0;
    public double transfer_set_left = 5317;
    public final double low_to_transfer = 330;
    public final double transfer_set_right = 0;
    public double low_set_left = 5090;
    public final double low_set_right = 0;

    public double under_bar_set_left;
    public double over_bar_set_left;
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
    public void establishPositions(DcMotorEx slideMotorLeft) {
        low_set_left = slideMotorLeft.getCurrentPosition();
        transfer_set_left = low_set_left+low_to_transfer;
        high_set_left = transfer_set_left+transfer_to_high;
        highest_set_left = high_set_left+high_to_highest;
        under_bar_set_left = high_set_left - 100;
        over_bar_set_left = high_set_left + 750;
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

    public void setHighest() {
        setTargetPositions(highest_set_left, highest_set_right);
    }

    public void setUnderBar() {
        setTargetPositions(under_bar_set_left, under_bar_set_left);
    }

    public void setOverBar() { setTargetPositions(over_bar_set_left, over_bar_set_left); }

    public void update(boolean PID, boolean rightSlide, double joyStickValue) {
        double elapsedTime = timer.seconds();
        if (PID) {
            control(leftTarget);
            if (rightSlide) {
                control(rightTarget);
            }
        } else {
            controlNoPID(slideMotorLeft, joyStickValue);
            if (rightSlide) {
                controlNoPID(slideMotorRight, joyStickValue);
            }
        }
    }

    private void control(double target) {
        double powerSlide;
        powerSlide = calculateMotorPowerWithPID(target);
        slideMotorLeft.setPower(powerSlide);
    }
    private void controlNoPID(DcMotorEx motor, double power) {
        motor.setPower(power);
    }


    private double calculateMotorPowerWithPID(double targetPosition) {
        double position = slideMotorLeft.getCurrentPosition();
        return lcontroller.calculate(position, targetPosition);
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

    public class SlidesHighest implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setTargetPositions(highest_set_left+250, highest_set_right);
            update(true, false, 0);

            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);


            if (Math.abs(posLeft - highest_set_left) > 50) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action slidesHighest() {
        return new SlidesHighest();
    }
    public class SlidesOverBar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setOverBar();
            update(true, false, 0);

            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);


            if (Math.abs(posLeft - leftTarget) > 100) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action slidesOverBar() {
        return new SlidesOverBar();
    }
    public class SlidesUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setHigh();
            update(true, false, 0);

            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);


            if (Math.abs(posLeft - leftTarget) > 50) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action slidesUp() {
        return new SlidesUp();
    }
    public class SlidesTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setTargetPositions(transfer_set_left+70, transfer_set_right);
            update(true, false, 0);

            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);


            if (Math.abs(posLeft - leftTarget) > 60) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action slidesTransfer() {
        return new SlidesTransfer();
    }
    public class SlidesUnderBar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setUnderBar();
            update(true, false, 0);

            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);


            if (Math.abs(posLeft - leftTarget) > 60) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action slidesUnderBar() {
        return new SlidesUnderBar();
    }
    public class SlidesDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setLow();
            update(true, false, 0);

            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);

            if (Math.abs(posLeft - leftTarget) > 100) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action slidesDown() {
        return new SlidesDown();
    }

    public class SlidesFullDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setLow();
            slideMotorLeft.setPower(-1);

            double posLeft = getCurrentLeftPosition();
            double posRight = getCurrentRightPosition();
            telemetryPacket.put("left slide pos", posLeft);
            telemetryPacket.put("right slide pos", posRight);

            if (posLeft != leftTarget) {
                return true;
            } else {
                slideMotorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action slidesFullDown() {
        return new SlidesFullDown();
    }



}