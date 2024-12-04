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

@TeleOp(name = "JellyTele", group = "OpMode")
public class JellyTele extends BaseOpMode {
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

    protected DriveMode driveMode = DriveMode.FIELDCENTRIC;


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
            updateSlideMode();
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
        MANUAL,
        ACTIVEINTAKE,
        TRANSFER,
        ACTIVEOUTTAKE
    }

    protected IntOutMode intOutMode = IntOutMode.MANUAL;

    private void updateIntOutModeFromGamepad() {
        if (GamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
            intOutMode = IntOutMode.ACTIVEINTAKE;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
            intOutMode = IntOutMode.ACTIVEOUTTAKE;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
            intOutMode = IntOutMode.TRANSFER;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
            intOutMode = IntOutMode.MANUAL;
        }
    }

    private double updateIntOutMode() {
        double intakeJoystickValue = 0;
        switch (intOutMode) {
            case MANUAL:
                intakeJoystickValue = applyDeadband(-GamepadEx2.getLeftY());
                armMotor.manualIntake(intakeJoystickValue/2);
                int intakePosition = armMotor.getTargetPosition();
                telemetry.addData("intake", intakePosition);
                break;
            case ACTIVEINTAKE:
                armMotor.setTargetPosition(-20); //PLACEHOLDER
                break;
            case ACTIVEOUTTAKE:
                outtakeRotatingArmServos.setOutput(-1, -1);
                break;
            case TRANSFER:
                outtakeRotatingArmServos.setOutput(0.45,0.45);
                armMotor.setTargetPosition(-5); // PLACEHOLDER
                break;
        }
        armMotor.update();
        return intakeJoystickValue;
    }

    private void updateDriveModeFromGamepad() {
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
            driveMode = DriveMode.FIELDCENTRIC;
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
        STATIONARY,
        UP,
        DOWN,
        FULLEXTEND,
        FULLRETRACT
    }

    protected SlideMode slideMode = SlideMode.STATIONARY;

    private void updateSlideModeFromGamepad() {
        if (!(GamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) && !(GamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER))) {
            slideMode = SlideMode.STATIONARY;
        } else if (GamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            slideMode = SlideMode.UP;
        } else if (GamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            slideMode = SlideMode.DOWN;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            slideMode = SlideMode.FULLEXTEND;
        } else if (GamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            slideMode = SlideMode.FULLRETRACT;
        }
    }

    private void updateSlideMode() {
        double slidePower = 0;
        switch (slideMode) {
            case STATIONARY:
                slidePower = 0;
                break;
            case UP:
                slidePower = 0.75;
                break;
            case DOWN:
                slidePower = -0.75;
                break;
            case FULLEXTEND:
                slides.setTargetPosition(1000);
                break;
            case FULLRETRACT:
                slides.setTargetPosition(0);
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
}


        //  private double[] DWFieldCentricDrive()
        // {
        //   double y = -applyDeadband(GamepadEx1.getLeftY());
        // double x = applyDeadband(GamepadEx1.getLeftX()) * STRAFE_ADJUSTMENT_FACTOR;
        //double r = applyDeadband(GamepadEx1.getRightX());
        // double botHeading = drive.pose.heading.toDouble();

        // double x2 = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        // double y2 = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        // return new double[]{
        //     y2 + x2 + r,
        //     y2 - x2 + r,
        //     y2 - x2 - r,
        //     y2 + x2 - r
        // };
        // }
//            double x2 = x*Math.cos(-Yaw)-y*Math.sin(-Yaw);
//            double y2 = x*Math.sin(-Yaw)+y*Math.cos(-Yaw);
//            double denominator = Math.max((Math.abs(y2)+Math.abs(x2)+Math.abs(r)),1);
//            frontLeftMotor.setPower((y2+x2+r)/denominator);
//            backLeftMotor.setPower((y2-x2+r)/denominator);
//            frontRightMotor.setPower((y2-x2-r)/denominator);
//            backRightMotor.setPower((y2+x2-r)/denominator);
//            if (GamepadEx1.wasJustPressed(GamepadKeys.Button.BACK)) {
//                imu.resetYaw();
//            }
//            double y3 = -GamepadEx2.getLeftY();
//            slideMotorLeft.setPower(y3);
//            slideMotorRight.setPower(y3);
//            slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            telemetry.update();