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
import static java.lang.Math.*;
//Jocelyn's playground
@TeleOp(name = "JellySmelly", group = "OpMode")
public class JellySmelly extends BaseOpMode {

    private final double PRECISION_MULTIPLIER_LOW = 0.35;
    private final double PRECISION_MULTIPLIER_HIGH = 0.7;
    private final double ENDGAME_ALERT_TIME = 110.0;
    private final double DEADBAND_VALUE = 0.02;
    private final double STRAFE_ADJUSTMENT_FACTOR = (14.0 / 13.0);
    private final int intakeArmDefault = 40;
    private final int intakeArmSlightRaise = 50;
    private final int intakeArmHandOff = 170;
    private final int slidesDefault= 0;
    private final int slidesSlightRaise = 0;
    private final int slidesFullRaise = 50;
    //change these values
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

    private enum LiftState {
        START,
        GRAB,
        HANDOFF,
        RAISE,
        DUMP,
        RETRACT
    }
    LiftState liftState = LiftState.START;

    protected JellySmelly.DriveMode driveMode = JellySmelly.DriveMode.FIELDCENTRIC;


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

            LiftFSM(timer);

            updateDriveMode(calculatePrecisionMultiplier());
            updateSlideMode();
            controlClaws();

            telemetry.update();
        }
    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
        GamepadEx2.readButtons();
        updateDriveModeFromGamepad();
        updateSlideModeFromGamepad();
    }

    private void LiftFSM(ElapsedTime timer) {
        switch (liftState) {
            case START:
                if (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                    intakeServo.closeClaw();
                    armMotor.setTargetPosition(intakeArmSlightRaise);
                    //slight rotate? have to test
                    liftState = LiftState.GRAB;
                }
                break;

            case GRAB:
                if ((Math.abs(armMotor.getCurrentPosition() - intakeArmSlightRaise)<10)||(GamepadEx2.wasJustPressed(GamepadKeys.Button.Y))) {
                    armMotor.setTargetPosition(intakeArmHandOff);
                    slides.setTransfer();
                    outtakeRotatingArmServos.armOuttakeIntake();
                    liftState = LiftState.HANDOFF;
                }
                break;

            case HANDOFF:
                if (((Math.abs(armMotor.getCurrentPosition()-intakeArmHandOff)<10)&&((Math.abs(slides.getCurrentLeftPosition() - slidesSlightRaise)<10))||(GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)))){
                    outtakeServo.closeClaw();
                    timer.reset();
                    liftState = LiftState.DUMP;
                }
                break;

            case RAISE:
                if (timer.seconds() > 0.5) {
                    intakeServo.openClaw();
                    slides.setHigh();
                    outtakeRotatingArmServos.armOuttakeDeposit();
                    liftState=LiftState.DUMP;
                }
                break;

            case DUMP:
                if (((Math.abs(slides.getCurrentLeftPosition() - slidesFullRaise)<10) || (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)))) {
                    outtakeServo.openClaw();
                    timer.reset();
                    liftState = LiftState.RETRACT;
                    //within threshold??
                }
                break;

            case RETRACT:
                if (timer.seconds() > 0.5) {
                    armMotor.setTargetPosition(intakeArmDefault);
                    slides.setLow();
                    outtakeRotatingArmServos.armOuttakeIntake();
                    liftState = LiftState.START;
                }
                break;
            default:
                liftState = LiftState.START;
        }

        if ((GamepadEx2.wasJustPressed(GamepadKeys.Button.X)) && (liftState != LiftState.START)) {
            liftState = LiftState.START;
        }
        telemetry.addData("state", liftState.toString());


    }

    private void updateDriveModeFromGamepad() {
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
            driveMode = JellySmelly.DriveMode.FIELDCENTRIC;
        } else if (GamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
            driveMode = JellySmelly.DriveMode.DWFIELDCENTRIC;
        } else if (GamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
            driveMode = JellySmelly.DriveMode.MECANUM;
        }
    }

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
        double y = -applyDeadband(GamepadEx1.getLeftY());
        return new double[]{
                y + x + r,
                y - x + r,
                y - x - r,
                y + x - r
        };
    }

    private double applyDeadband(double joystickValue) {
        double sign = Math.signum(joystickValue);
        return joystickValue + (-sign * DEADBAND_VALUE);
    }

    private double[] FieldCentricDrive() {
        double y = -applyDeadband(GamepadEx1.getLeftY());
        double x = applyDeadband(GamepadEx1.getLeftX()) * STRAFE_ADJUSTMENT_FACTOR;
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
        return MAX_SCALE;
    }

    private enum SlideMode {
        FULLEXTEND,
        FULLRETRACT,
        MANUAL
    }

    protected JellySmelly.SlideMode slideMode = JellySmelly.SlideMode.MANUAL;

    private void updateSlideModeFromGamepad() {
        if (GamepadEx2.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            slideMode = JellySmelly.SlideMode.MANUAL;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slideMode = JellySmelly.SlideMode.FULLEXTEND;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slideMode = JellySmelly.SlideMode.FULLRETRACT;
        }
    }

    private void updateSlideMode() {
        double slidePower = 0;
        switch (slideMode) {
            case MANUAL:
                if (GamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slidePower = 0.5;
                }
                if (GamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slidePower = -0.5;
                }
            case FULLEXTEND:
                slides.setHigh();
                break;
            case FULLRETRACT:
                slides.setLow();
                break;
        }
        slideMotorLeft.setPower(slidePower);
        slideMotorRight.setPower(slidePower);
        slides.update();
        double leftPosition = slideMotorLeft.getCurrentPosition();
        double rightPosition = slideMotorRight.getCurrentPosition();
        telemetry.addData("Left", leftPosition);
        telemetry.addData("Right", rightPosition);
    }
    private void controlClaws() {
        if (GamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1) {
            intakeServo.openClaw();
        } else {
            intakeServo.closeClaw();
        }
        if (GamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1) {
            outtakeServo.openClaw();
        } else {
            outtakeServo.closeClaw();
        }
    }
}
