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
    GamepadEx GamepadEx2;
    double y1 = 0;
    double y2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);
        initHardware(false);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);


        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            readGamepadInputs();
            controlClawLeft();
            controlClawRight();
            telemetry.update();
        }
    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
        GamepadEx2.readButtons();

    }

    private void controlClawLeft() {
        if (gamepad1.dpad_left)
        {
            y1 -= 0.0005;
        }
        if (gamepad1.dpad_right)
        {
            y1 += 0.0005;
        }
        intakeServo.setClawPosDouble(y1);
        telemetry.addData("intake clawposition:", intakeServo.getClawPosition());

    }

    private void controlClawRight() {
        if (gamepad2.dpad_left)
        {
            y2 -= 0.0005;
        }
        if (gamepad2.dpad_right)
        {
            y2 += 0.0005;
        }
        outtakeServo.setClawPosDouble(y2);
        telemetry.addData("outtake clawposition:", outtakeServo.getClawPosition());
        telemetry.addData("outtake clawposition:", outtakeServo.getClawPosition());

    }
}
