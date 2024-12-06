package org.firstinspires.ftc.teamcode.Tuners;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class slidesTuner extends LinearOpMode
{
    private PIDController controller;

    public static double p = 0.03, i = 0.0061, d = 0.0004;
    public static int target = 300; //who the heck knows
    private DcMotorEx slideMotorLeft;
    private DcMotorEx slideMotorRight;

    @Override
    public void runOpMode()
    {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //slideMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //slideMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slideMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive())
        {
            controller.setPID(p, i, d);
            int position = slideMotorLeft.getCurrentPosition();
            double power = controller.calculate(position, target);
            slideMotorLeft.setPower(power);
            slideMotorRight.setPower(power);
            telemetry.addData("pos ", position);
            telemetry.addData("target ", target);
            telemetry.update();
        }
    }
}