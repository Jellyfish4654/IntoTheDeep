package org.firstinspires.ftc.teamcode.Tuners;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;

@Config
@TeleOp
public class slidesManualTuner extends BaseOpMode
{


    private DcMotorEx slideMotorLeft;

    GamepadEx GamepadEx1;
    private DcMotorEx slideMotorRight;

    GamepadEx GamepadEx2;


    @Override
    public void runOpMode()
    {

        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);
        initHardware();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        //slideMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //slideMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slideMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive())
        {
            readGamepadInputs();
            controlSlideLeft();
            controlSlideRight();

            telemetry.update();
        }


    }

    private void readGamepadInputs() {
        GamepadEx1.readButtons();
    }

    private void controlSlideLeft() {
        if (gamepad1.dpad_left)
        {

            slideMotorRight.setPower(1);
        }
        if (gamepad1.dpad_right)
        {

            slideMotorRight.setPower(-1);
        }
        telemetry.addData("slide left :", slideMotorRight.getCurrentPosition());


    }

    private void controlSlideRight() {
        if (gamepad2.dpad_left)
        {

            slideMotorLeft.setPower(1);
        }
        if (gamepad2.dpad_right)
        {

            slideMotorLeft.setPower(-1);
        }
        telemetry.addData("slide right :", slideMotorLeft.getCurrentPosition());


    }
}