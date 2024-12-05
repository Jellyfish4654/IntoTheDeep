package org.firstinspires.ftc.teamcode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Double Servo Test", group = "Test")
public class doubleServoTuner extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        final Servo outtakeLeftServo;
        final Servo outtakeRightServo;
        outtakeLeftServo = hardwareMap.get(Servo.class, "outtakeArmServoLeft");
        outtakeRightServo = hardwareMap.get(Servo.class, "outtakeArmServoRight");

//        outtakeLeftServo.setDirection(Servo.Direction.REVERSE);
        double position = 0; // Initialize to midpoint

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("position: ", position);
            telemetry.update();

            outtakeLeftServo.setPosition(position);
            outtakeRightServo.setPosition(position);

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
                position = 0.50;
            }
            else if (gamepad1.b)
            {
                position = -1;
            }
        }
    }
}
