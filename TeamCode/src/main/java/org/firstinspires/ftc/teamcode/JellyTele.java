package org.firstinspires.ftc.teamcode;

import static android.icu.util.UniversalTimeScale.MAX_SCALE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Framework.SlewRateLimiter;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import com.arcrobotics.ftclib.controller.PIDController;

@TeleOp(name = "JellyTele", group = "OpMode")
public class JellyTele extends BaseOpMode {
    private final double PRECISION_MULTIPLIER_LOW = 0.35;
    private final double PRECISION_MULTIPLIER_HIGH = 0.7;
    private final double ENDGAME_ALERT_TIME = 110.0;
    private final double DEADBAND_VALUE = 0.02;
    private final double STRAFE_ADJUSTMENT_FACTOR = (14.0 / 13.0);
    private double resetHeading = 0;
    private final SlewRateLimiter[] slewRateLimiters = new SlewRateLimiter[4];
    private double defaultSlidePower = 0;
    private MecanumDrive drive;
    GamepadEx GamepadEx1;
    GamepadEx GamepadEx2;

    private enum DriveMode {
        MECANUM,
        FIELDCENTRIC,
        DWFIELDCENTRIC
    }

    protected DriveMode driveMode = DriveMode.MECANUM;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initializeSlewRateLimiters();
        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            readGamepadInputs();
            updateDriveMode(calculatePrecisionMultiplier());
            updateIntOutMode();
            updateClawsManual();
            updateSlideMode();
            controlIntake();
            telemetry.update();
        }
    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
        GamepadEx2.readButtons();
        updateDriveModeFromGamepad();
        updateIntOutModeFromGamepad();
        updateSlideModeFromGamepad();
    }

    private enum IntOutMode {
        ACTIVEINTAKE,
        ACTIVEOUTTAKE,
        TRANSFER
    }
    protected IntOutMode intOutMode = IntOutMode.ACTIVEINTAKE;

    private void updateClawsManual() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            outtakeServo.closeClaw();
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            outtakeServo.openClaw();
        }
    }
    /*
    private enum ClawMode {
        OPEN,
        CLOSE
    }
    protected ClawMode clawMode = ClawMode.OPEN;
    private void updateClawMode() {
        switch (clawMode) {
            case OPEN:
                outtakeServo.openClaw();
                break;
            case CLOSE:
                outtakeServo.closeClaw();
                break;
        }
    }
    private void updateClawModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            if (clawMode == ClawMode.OPEN) {
                clawMode = ClawMode.CLOSE;
            } else if (clawMode == ClawMode.CLOSE) {
                clawMode = ClawMode.OPEN;
            }
        }
    }*/

    private void updateIntOutModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
            intOutMode = IntOutMode.ACTIVEINTAKE;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
            intOutMode = IntOutMode.ACTIVEOUTTAKE;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
            intOutMode = IntOutMode.TRANSFER;
        }
    }

    private void updateIntOutMode() {
        double intakeJoystickValue = 0;
        switch (intOutMode) {
            case ACTIVEINTAKE:
                //armMotor.intakePos(); //OR 255
                //intakeRotatingArmServos.armIntakePosition();
                slides.setTransfer();
                break;
            case ACTIVEOUTTAKE:
                outtakeRotatingArmServos.armOuttakeDeposit();
                slides.setHigh();
                break;
            case TRANSFER:
                //armMotor.transferPos(); //OR 82
                //intakeRotatingArmServos.armIntakeDeposit();
                slides.setTransfer();
                outtakeRotatingArmServos.armOuttakeIntake();
                break;
        }
        slides.enablePID();
        slides.update();
        outtakeRotatingArmServos.setOutput();
        telemetry.addData("state:", intOutMode.toString());
        telemetry.addData("slides right pos:", slides.getCurrentRightPosition());
        telemetry.addData("slides left pos:", slides.getCurrentLeftPosition());
        telemetry.addData("intakeArm left pos:", intakeRotatingArmServos.getPositionLeft());
        telemetry.addData("intakeArm right pos:", intakeRotatingArmServos.getPositionRight());
    }
    private void updateDriveModeFromGamepad() {
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
            driveMode = DriveMode.MECANUM;
        } else if (GamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
            driveMode = DriveMode.DWFIELDCENTRIC;
        } else if (GamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
            driveMode = DriveMode.MECANUM;
        }
    }

    private void updateDriveMode(double precisionMultiplier) {
        double[] motorSpeeds;
        switch (driveMode) {
            case MECANUM:
                motorSpeeds = MecanumDrive();
                break;
            case FIELDCENTRIC:
                motorSpeeds = MecanumDrive();
                break;
            //case DWFIELDCENTRIC:
            default:
                motorSpeeds = MecanumDrive();
                break;
            //  add dwfieldcentric later;
        }
        drivetrain.setMotorSpeeds(precisionMultiplier, motorSpeeds);

    }

    private double[] MecanumDrive() {
        double r = applyDeadband(GamepadEx1.getRightX());
        double x = applyDeadband(GamepadEx1.getLeftX()) * STRAFE_ADJUSTMENT_FACTOR;
        double y = -applyDeadband(GamepadEx1.getLeftY());
        return new double[]{
                0.6*(y + x + r),
                0.6*(y - x + r),
                0.6*(y - x - r),
                0.6*(y + x - r)
        };
    }

    private double applyDeadband(double joystickValue) {
        double sign = Math.signum(joystickValue);
        return joystickValue + (-sign * DEADBAND_VALUE);
    }

    private double[] FieldCentricDrive() {
        double y = -applyDeadband(GamepadEx1.getLeftY());
        double x = applyDeadband(GamepadEx1.getLeftX()) * STRAFE_ADJUSTMENT_FACTOR;
        double r = -applyDeadband(GamepadEx1.getRightX());
        double Yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - resetHeading;

        double x2 = x * Math.cos(-Yaw) - y * Math.sin(-Yaw);
        double y2 = x * Math.sin(-Yaw) + y * Math.cos(-Yaw);
        return new double[]{
                y2 + x2 + r,
                y2 - x2 + r,
                y2 - x2 - r,
                y2 + x2 - r

        };
    }


    private void applySlewRateLimit(double[] powers, double rate) {
        for (int i = 0; i < slewRateLimiters.length; i++) {
            slewRateLimiters[i].setRate(rate);
            powers[i] = slewRateLimiters[i].calculate(powers[i]);
        }
    }

    private void initializeSlewRateLimiters() {
        for (int i = 0; i < slewRateLimiters.length; i++) {
            slewRateLimiters[i] = new SlewRateLimiter(1.0);
        }
    }


    private double calculatePrecisionMultiplier() {
        if (GamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            return PRECISION_MULTIPLIER_LOW;
        } else if (GamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            return PRECISION_MULTIPLIER_HIGH;
        }
        return MAX_SCALE;
    }

    private enum SlideMode {
        MANUAL,
        FULLEXTEND,
        FULLRETRACT
    }

    protected SlideMode slideMode = SlideMode.FULLRETRACT;

    private void updateSlideModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slideMode = SlideMode.FULLRETRACT;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slideMode = SlideMode.FULLEXTEND;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            slideMode = SlideMode.MANUAL;
        }
    }

    private void updateSlideMode() {
       double slidePower = defaultSlidePower;
        switch (slideMode) {
            case MANUAL:
                slides.disablePID();
                if (GamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slidePower = 1;
                }
                if (GamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slidePower = -1;
                }
                if (GamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                    defaultSlidePower = -1;
                }
                slideMotorLeft.setPower(slidePower);
                slideMotorRight.setPower(slidePower);
            case FULLEXTEND:
                slides.enablePID();
                slides.setHigh();
                break;
            case FULLRETRACT:
                slides.enablePID();
                slides.setLow();
                break;
        }
        slides.update();
        double leftPosition = slideMotorLeft.getCurrentPosition();
        double rightPosition = slideMotorRight.getCurrentPosition();
        telemetry.addData("Left", leftPosition);
        telemetry.addData("Right", rightPosition);
    }
    private void controlIntake() {
        double intakeJoystickValue = Math.abs((applyDeadband(GamepadEx2.getLeftY())));
        intakeRotatingArmServos.setOutput(intakeJoystickValue/2+0.4);
    }
}