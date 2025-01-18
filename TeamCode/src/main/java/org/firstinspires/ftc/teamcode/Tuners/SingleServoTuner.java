package org.firstinspires.ftc.teamcode.Tuners;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Single Servo Test", group = "Test")
public class SingleServoTuner extends LinearOpMode
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		final Servo outtakeLeftServo;
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		outtakeLeftServo = hardwareMap.get(Servo.class, "outtakeArmServoLeft");
		GamepadEx GamepadEx1;
		GamepadEx1 = new GamepadEx(gamepad1);
//        outtakeLeftServo.setDirection(Servo.Direction.REVERSE);
		double position = outtakeLeftServo.getPosition();
		// Initialize to midpoint

		waitForStart();

		while (opModeIsActive())
		{
			GamepadEx1.readButtons();
			telemetry.addData("target position", position);
			telemetry.addData("position", outtakeLeftServo.getPosition());
			telemetry.update();
			outtakeLeftServo.setDirection(Servo.Direction.REVERSE);
			outtakeLeftServo.setPosition(position);

			if (GamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
			{
				position -= 0.01;
			}
			if (GamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
			{
				position += 0.01;
			}

			if (gamepad1.a)
			{
				position = 0.5;
			}
			else if (gamepad1.b)
			{
				position = 0.79;
			}
		}
	}
}
