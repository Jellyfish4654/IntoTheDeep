package org.firstinspires.ftc.teamcode.Tuners;

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

//        outtakeLeftServo.setDirection(Servo.Direction.REVERSE);
		double position = outtakeLeftServo.getPosition();
		// Initialize to midpoint

		waitForStart();

		while (opModeIsActive())
		{

			telemetry.addData("target position", position);
			telemetry.addData("position", outtakeLeftServo.getPosition());
			telemetry.update();

			outtakeLeftServo.setPosition(position);

			if (gamepad1.dpad_left)
			{
				position -= 0.01;
			}
			if (gamepad1.dpad_right)
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
