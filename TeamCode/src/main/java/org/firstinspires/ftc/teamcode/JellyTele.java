package org.firstinspires.ftc.teamcode;

import static android.icu.util.UniversalTimeScale.MAX_SCALE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Framework.SlewRateLimiter;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import com.arcrobotics.ftclib.controller.PIDController;

@TeleOp(name = "JellyTele", group = "OpMode")
public class JellyTele extends BaseOpMode {
    private VoltageSensor voltageSensor;
    private final double PRECISION_MULTIPLIER_LOW = 0.35;
    private final double PRECISION_MULTIPLIER_HIGH = 0.7;
    private final double ENDGAME_ALERT_TIME = 110.0;
    private final double DEADBAND_VALUE = 0.02;
    private final double STRAFE_ADJUSTMENT_FACTOR = (14.0 / 13.0);
    private double resetHeading = 0;
    private final SlewRateLimiter[] slewRateLimiters = new SlewRateLimiter[4];
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
            updateOuttakeMode();
            updateClawsManual();
            updateWrist();
            updateSlideMode();
            controlExtendo();
            telemetry.update();
        }
    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
        GamepadEx2.readButtons();
        updateDriveModeFromGamepad();
        updateOuttakeModeFromGamepad();
        updateSlideModeFromGamepad();
    }

    private enum OuttakeMode {
        ACTIVEOUTTAKE,
        TRANSFER
    }
    protected OuttakeMode outtakeMode = OuttakeMode.ACTIVEOUTTAKE;

    private void updateClawsManual() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            intakeServo.clawToggle();
            outtakeServo.clawToggle();
        }

        telemetry.addData("claw intake pos:", intakeServo.getClawPosition());
        telemetry.addData("claw outtake pos:", outtakeServo.getClawPosition());
    }

    private void updateWrist() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) ) {
            wrist.setPosDown();
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            wrist.setPosUp();
        }
    }

    private void updateOuttakeModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
            outtakeMode = OuttakeMode.ACTIVEOUTTAKE;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
            outtakeMode = OuttakeMode.TRANSFER;
        }
    }

    private void updateOuttakeMode() {
        switch (outtakeMode) {
            case ACTIVEOUTTAKE:
                outtakeRotatingArmServos.armOuttakeDeposit();
                break;
            case TRANSFER:
                outtakeRotatingArmServos.armOuttakeIntake();
                break;
        }
        outtakeRotatingArmServos.setOutput();
        telemetry.addData("state:", outtakeMode.toString());
        telemetry.addData("outtake servo position", outtakeRotatingArmServos.getCurrentPosition());
        telemetry.addData("slides right pos:", slides.getCurrentRightPosition());
        telemetry.addData("slides left pos:", slides.getCurrentLeftPosition());
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            outtakeRotatingArmServos.subtractServoPos();
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            outtakeRotatingArmServos.addServoPos();
        }
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
        double y = applyDeadband(GamepadEx1.getLeftY());

        double sum = ((Math.abs(y))+(Math.abs(x))+(Math.abs(r)));
        double denominator = Math.max(sum, 1);

        return new double[]{
                0.6*(y + x + r)/denominator,
                0.6*(y - x + r)/denominator,
                0.6*(y - x - r)/denominator,
                0.6*(y + x - r)/denominator
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
        HIGH,
        LOW,
        TRANSFER,
        MANUAL,
        HANGPREP,
        HANG
    }

    protected SlideMode slideMode = SlideMode.MANUAL;

    private void updateSlideModeFromGamepad() {
//        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
//            slideMode = SlideMode.LOW;
//        }
//        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
//            slideMode = SlideMode.HIGH;
//        }
//        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
//            slideMode = SlideMode.TRANSFER;
//        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
            slideMode = SlideMode.MANUAL;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (slideMode == SlideMode.HANGPREP) {
                slideMode = SlideMode.HANG;
            } else {
                slideMode = SlideMode.HANGPREP;
            }
        }
    }

    private void updateSlideMode() {
       double slidePower = 0;
        switch (slideMode) {
            case HIGH:
                slides.setHigh();
                slides.update(true, false,0);
                break;
            case LOW:
                slides.setLow();
                //left = -27, right = -5
                slides.update(true, false,0);
                break;
            case TRANSFER:
                slides.setTransfer();
                slides.update(true, false,0);
            case MANUAL:
                slidePower = -applyDeadband(GamepadEx2.getRightY()/2);
                slides.update(false, false, slidePower);
            case HANGPREP:
                slidePower = -applyDeadband(GamepadEx2.getRightY()/2);
                slides.update(false, true, slidePower);
            case HANG:
                slidePower = -1;
                slides.update(false, true, slidePower);
        }
        double leftPosition = slideMotorLeft.getCurrentPosition();
        double rightPosition = slideMotorRight.getCurrentPosition();
        telemetry.addData("Left", leftPosition);
        telemetry.addData("Right", rightPosition);
    }

    private void controlExtendo() {
        double extendoJoystickValue = -(applyDeadband(GamepadEx2.getLeftY()));
        Extendo.setPower(extendoJoystickValue);
    }

}