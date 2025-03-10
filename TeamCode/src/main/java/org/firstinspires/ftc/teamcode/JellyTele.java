package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Framework.Algorithms.SlewRateLimiter;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp(name = "JellyTele", group = "OpMode")
public class JellyTele extends BaseOpMode {
    private VoltageSensor voltageSensor;
    boolean leftTrigger = false;
    boolean rightTrigger = false;
    private final double PRECISION_MULTIPLIER_LOW = 0.35;
    private final double PRECISION_MULTIPLIER_HIGH = 0.2;
    private final double ENDGAME_ALERT_TIME = 110.0;
    private final double DEADBAND_VALUE = 0.02;
    private final double STRAFE_ADJUSTMENT_FACTOR = (14.0 / 13.0);
    private double resetHeading = 0;
    private final SlewRateLimiter[] slewRateLimiters = new SlewRateLimiter[4];
    private MecanumDrive drive;
    GamepadEx GamepadEx1;
    GamepadEx GamepadEx2;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(false);
        initializeSlewRateLimiters();
        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
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
            updateExtendoMode();
            resetPositions();
            telemetry.update();
        }
    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
        GamepadEx2.readButtons();
        updateDriveModeFromGamepad();
        updateOuttakeModeFromGamepad();
        updateSlideModeFromGamepad();
        updateExtendoModeFromGamepad();
    }

    private void updateClawsManual() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            intakeServo.clawToggle();
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
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

    private enum OuttakeMode {
        GRAB,
        TRANSFER,
        CHAMBER,
        BASKET
    }
    protected OuttakeMode outtakeMode = OuttakeMode.TRANSFER;
    private void updateOuttakeModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
            outtakeMode = OuttakeMode.BASKET;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
            outtakeMode = OuttakeMode.TRANSFER;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
            outtakeMode = OuttakeMode.GRAB;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
            outtakeMode = OuttakeMode.CHAMBER;
        }
    }

    private void updateOuttakeMode() {
        switch (outtakeMode) {
            case GRAB:
                outtakeRotatingArm.armOuttakeGrab();
                break;
            case TRANSFER:
                outtakeRotatingArm.armOuttakeIntake();
                break;
            case BASKET:
                outtakeRotatingArm.armOuttakeDeposit();
                break;
            case CHAMBER:
                outtakeRotatingArm.armOuttakeChamber();
                break;
        }
        outtakeRotatingArm.setOutput();
        telemetry.addData("state:", outtakeMode.toString());
        telemetry.addData("outtake servo position", outtakeRotatingArm.getCurrentPosition());
        telemetry.addData("slides right pos:", slides.getCurrentRightPosition());
        telemetry.addData("slides left pos:", slides.getCurrentLeftPosition());
    }
    private void updateDriveModeFromGamepad() {
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
            driveMode = DriveMode.MECANUM;
        }
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
            driveMode = DriveMode.FIELDCENTRIC;
        }
    }

    private enum DriveMode {
        MECANUM,
        FIELDCENTRIC
    }

    protected DriveMode driveMode = DriveMode.MECANUM;

    private void updateDriveMode(double precisionMultiplier) {
        double[] motorSpeeds;
        switch (driveMode) {
            case MECANUM:
                motorSpeeds = MecanumDrive();
                break;
            case FIELDCENTRIC:
                motorSpeeds = FieldCentricDrive();
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
                (y + x + r)/denominator,
                (y - x + r)/denominator,
                (y - x - r)/denominator,
                (y + x - r)/denominator
        };
    }

    private double applyDeadband(double joystickValue) {
        double sign = Math.signum(joystickValue);
        return joystickValue + (-sign * DEADBAND_VALUE);
    }

    private double[] FieldCentricDrive() {
        double y = -applyDeadband(GamepadEx1.getLeftY());
        double x = -applyDeadband(GamepadEx1.getLeftX()) * STRAFE_ADJUSTMENT_FACTOR;
        double r = applyDeadband(GamepadEx1.getRightX());
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
        return 1;
    }

    protected enum SlideMode {
        BASKET,
        HIGH,
        LOW,
        TRANSFER,
        MANUAL,
        HANGPREP,
        HANG
    }

    protected SlideMode slideMode = SlideMode.MANUAL;

    private void updateSlideModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
            slideMode = SlideMode.BASKET;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
            slideMode = SlideMode.LOW;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            slideMode = SlideMode.MANUAL;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
            slideMode = SlideMode.HIGH;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
            slideMode = SlideMode.TRANSFER;
        }
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
            slideMode = SlideMode.HANGPREP;
        }
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
            slideMode = SlideMode.HANG;
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
                slides.update(true, false,0);
                break;
            case TRANSFER:
                slides.setTransfer();
                slides.update(true, false,0);
                break;
            case MANUAL:
                slidePower = -applyDeadband(GamepadEx2.getRightY());
                if (Math.signum(slidePower) == -1) {
                    slidePower *= 0.6;
                }
                slides.update(false, false, slidePower);
                break;
            case HANGPREP:
                slidePower = -applyDeadband(GamepadEx2.getRightY());
                if (Math.signum(slidePower) == -1) {
                    slidePower *= 0.6;
                }
                slides.update(false, true, slidePower);
                break;
            case BASKET:
                slides.setHighest();
                slides.update(true, false,0);
                break;
            case HANG:
                slides.setUnderBar();
                slides.update(true, true, 0);
                break;
        }
        double leftPosition = slideMotorLeft.getCurrentPosition();
        double rightPosition = slideMotorRight.getCurrentPosition();
        telemetry.addData("Left", leftPosition);
        telemetry.addData("Right", rightPosition);
        telemetry.addData("slides state:", slideMode);
        telemetry.addData("slides power:", slidePower);
    }

    protected enum ExtendoMode {
        IN,
        MANUAL
    }

    protected ExtendoMode extendoMode = ExtendoMode.MANUAL;

    private void updateExtendoModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            extendoMode = ExtendoMode.IN;
        }
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            extendoMode = ExtendoMode.MANUAL;
        }
    }
    private void updateExtendoMode() {
        switch (extendoMode) {
            case IN:
                extendo.setRetract();
                extendo.update();
                break;
            case MANUAL:
                controlExtendo();
                break;
        }
        telemetry.addData("extendo pos:", Extendo.getCurrentPosition());
    }
    private void controlExtendo() {
        double extendoJoystickValue = -(applyDeadband(GamepadEx2.getLeftY()));
        Extendo.setPower(extendoJoystickValue);
    }



    private boolean leftTriggerPressed() {
        boolean currentTrigger;
        if (GamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            currentTrigger = true;
        } else {
            currentTrigger = false;
        }
        if (currentTrigger != leftTrigger) {
            leftTrigger = currentTrigger;
            return true;
        } else {
            return false;
        }
    }
    private boolean rightTriggerPressed() {
        boolean currentTrigger;
        if (GamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            currentTrigger = true;
        } else {
            currentTrigger = false;
        }
        if (currentTrigger != rightTrigger) {
            rightTrigger = currentTrigger;
            return true;
        } else {
            return false;
        }
    }

    public void resetPositions() {
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP) || GamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) || GamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || GamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slides.establishPositions(slideMotorLeft);
            extendo.establishPositions();
        }
    }

}