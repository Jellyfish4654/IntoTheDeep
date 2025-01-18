package org.firstinspires.ftc.teamcode.Tuners;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;

@TeleOp(name = "ClawTuner", group = "OpMode")
public class clawTuner extends BaseOpMode {
    GamepadEx GamepadEx1;
    private final double DEADBAND_VALUE = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx1 = new GamepadEx(gamepad1);
        initHardware();
//        //initializeSlewRateLimiters();
//        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

//        intakeClaw = new Claw(hardwareMap.get(Servo.class, "intakeServo"));

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            readGamepadInputs();
            controlClaw();
            telemetry.update();
        }
    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
    }

    private void controlClaw() {
        double y = -applyDeadband(GamepadEx1.getLeftY());
        if (gamepad1.dpad_left)
        {
            y -= 0.01;
        }
        if (gamepad1.dpad_right)
        {
            y += 0.01;
        }
        outtakeServo.setClawPos(y);
        telemetry.addData("clawposition:", outtakeServo.getClawPosition());

    }

    private double applyDeadband(double joystickValue) {
        double sign = Math.signum(joystickValue);
        return joystickValue + (-sign * DEADBAND_VALUE);
    }
}
