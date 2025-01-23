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
    private PIDController lcontroller;
    private PIDController rcontroller;

    public static double pleft = 0, ileft = 0, dleft = 0;
    public static double pright = 0, iright = 0, dright = 0;
    public static int leftTarget = -1852; //who the heck knows
    public static int rightTarget = 0;
    private DcMotorEx slideMotorLeft;
    private DcMotorEx slideMotorRight;

    @Override
    public void runOpMode()
    {
        lcontroller = new PIDController(pleft, ileft, dleft);
        rcontroller = new PIDController(pleft, ileft, dleft);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        //slideMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //slideMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slideMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive())
        {
            lcontroller.setPID(pleft, ileft, dleft);
            rcontroller.setPID(pright, iright, dright);
            int Lposition = slideMotorLeft.getCurrentPosition();
            int Rposition = slideMotorRight.getCurrentPosition();
            double lpower = lcontroller.calculate(Lposition, leftTarget);
            double rpower = rcontroller.calculate(Rposition, rightTarget);

            slideMotorLeft.setPower(lpower);
            slideMotorRight.setPower(rpower);
            telemetry.addData("left pos ", Lposition);
            telemetry.addData("right pos ", Rposition);
            telemetry.addData("left target ", leftTarget);
            telemetry.addData("right target ", rightTarget);
            telemetry.update();
        }
    }
}