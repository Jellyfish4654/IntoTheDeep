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
    GamepadEx GamepadEx1 = new GamepadEx(gamepad1);
    GamepadEx GamepadEx2 = new GamepadEx(gamepad2);
    IMU imu;

    private enum DriveMode {
        MECANUM,
        FIELDCENTRIC,
        DWFIELDCENTRIC
    }

    protected DriveMode driveMode = DriveMode.DWFIELDCENTRIC;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initializeSlewRateLimiters();
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            readGamepadInputs();
            controlIntake();
            controlSlideMotors();
            controlOuttake();
            updateDriveMode(calculatePrecisionMultiplier());
            telemetry.update();
        }
    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
        GamepadEx2.readButtons();
        updateDriveModeFromGamepad();
    }

    private void controlIntake() {
        controlIntakeMotor();
        if (gamepad2.y) {
            intakeActivePosition();
        } else if (gamepad2.b) {
            outtakeActivePosition();
        } else if (gamepad2.a) {
            intakeOuttakeTransfer();
        }
    }
    private void controlIntakeMotor() {
        double joystickValue = applyDeadband(-gamepad2.left_stick_y);
        armMotor.setPower(joystickValue);
    }
    private void intakeActivePosition() {
        //set intake to position (button y)
    }
    private void outtakeActivePosition() {
        //set outtake to position (button b)
    }
    private void intakeOuttakeTransfer() {
        //move intake, outtake, and slides to the correct places (button a)
    }

    private void controlOuttake() {

    }

    private void updateDriveModeFromGamepad() {
        if (GamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
            driveMode = DriveMode.FIELDCENTRIC;
        } else if (GamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
            driveMode = DriveMode.DWFIELDCENTRIC;
        } else if (GamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
            driveMode = DriveMode.MECANUM;
        }
        //resetIMU();
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
        setMotorSpeeds(precisionMultiplier, motorSpeeds);
    }

    private double[] MecanumDrive() {
        double y = applyDeadband(gamepad1.right_stick_x);
        double x = applyDeadband(gamepad1.left_stick_x) * STRAFE_ADJUSTMENT_FACTOR;
        double r = -applyDeadband(gamepad1.left_stick_y);
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
        double y = -applyDeadband(gamepad1.left_stick_y);
        double x = applyDeadband(gamepad1.left_stick_x) * STRAFE_ADJUSTMENT_FACTOR;
        double r = applyDeadband(gamepad1.right_stick_x);
        double Yaw = imuSensor.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - resetHeading;

        double x2 = x * Math.cos(-Yaw) - y * Math.sin(-Yaw);
        double y2 = x * Math.sin(-Yaw) + y * Math.cos(-Yaw);
        return new double[]{
                y2 + x2 + r,
                y2 - x2 + r,
                y2 - x2 - r,
                y2 + x2 - r

        };
    }

    protected void setMotorSpeeds(double multiplier, double[] powers) {
        applyPrecisionAndScale(multiplier, powers);
        int averagePosition = (slideMotorLeft.getCurrentPosition() + slideMotorRight.getCurrentPosition() / 2);
        double rate = 1.0;
        if (averagePosition >= 2000) {
            rate = 0.99 - ((double) (averagePosition - 2000) / 10) * 0.0005;
            rate = Math.max(rate, 0);
            applySlewRateLimit(powers, rate);
        }
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setPower(powers[i]);
        }
    }

    private void applyPrecisionAndScale(double multiplier, double[] powers) {
        for (int i = 0; i < powers.length; i++) {
            powers[i] *= multiplier;
        }

        double maxPower = findMaxPower(powers);
        double scale = maxPower > MAX_SCALE ? MAX_SCALE / maxPower : 1.0;

        for (int i = 0; i < powers.length; i++) {
            powers[i] *= scale;
        }
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

    private double findMaxPower(double[] powers) {
        double max = 0;
        for (double power : powers) {
            max = Math.max(max, Math.abs(power));
        }
        return max;
    }
    private double calculatePrecisionMultiplier()
    {
        if (gamepad1.left_bumper)
        {
            return PRECISION_MULTIPLIER_LOW;
        }
        else if (gamepad1.right_bumper)
        {
            return PRECISION_MULTIPLIER_HIGH;
        }
        return MAX_SCALE;
    }
    private void controlSlideMotors() {
        double slidePower = 0;
        if (gamepad2.left_bumper) {
           slidePower = 1;
        } else if (gamepad2.right_bumper) {
            slidePower = -1;
        } else if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            slidePower = 0;
        }
        if (applyDeadband(slidePower) != 0) {
            slideMotorLeft.setPower(slidePower);
            slideMotorRight.setPower(slidePower);
        }
        double leftpos = slideMotorLeft.getCurrentPosition();
        double rightpos = slideMotorRight.getCurrentPosition();
        telemetry.addData("Left",leftpos);
        telemetry.addData("Right", rightpos);


        //  private double[] DWFieldCentricDrive()
        // {
        //   double y = -applyDeadband(gamepad1.left_stick_y);
        // double x = applyDeadband(gamepad1.left_stick_x) * STRAFE_ADJUSTMENT_FACTOR;
        //double r = applyDeadband(gamepad1.right_stick_x);
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
    }
}
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
//            double y3 = -gamepad2.right_stick_y;
//            slideMotorLeft.setPower(y3);
//            slideMotorRight.setPower(y3);
//            slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            telemetry.update();