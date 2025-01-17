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
public class extendoTuner extends LinearOpMode
{
    private PIDController controller;
    public static double p = 0.03, i = 0.0061, d = 0.0004;
    public static int target = 300; //who the heck knows
    private DcMotorEx extendo;

    @Override
    public void runOpMode()
    {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        waitForStart();

        while (opModeIsActive())
        {
            controller.setPID(p, i, d);
            int position = extendo.getCurrentPosition();
            double power = controller.calculate(position, target);

            extendo.setPower(power);
            telemetry.addData("pos ", position);
            telemetry.addData("target ", target);
            telemetry.update();
        }
    }
}