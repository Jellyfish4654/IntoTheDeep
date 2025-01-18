package org.firstinspires.ftc.teamcode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "wrist tunnn", group = "Test")
public class wristTuner extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        final Servo wristServo;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        wristServo = hardwareMap.get(Servo.class, "wristServo");

//        outtakeLeftServo.setDirection(Servo.Direction.REVERSE);
        double position = wristServo.getPosition();
        // Initialize to midpoint

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("target position", position);
            telemetry.addData("position", wristServo.getPosition());
            telemetry.update();

            wristServo.setPosition(position);

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
                position = 0.33;
            }
            else if (gamepad1.x)
            {
                position = 1;
            }
        }
    }
}
