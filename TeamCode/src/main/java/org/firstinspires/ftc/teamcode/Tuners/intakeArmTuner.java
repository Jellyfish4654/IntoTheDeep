package org.firstinspires.ftc.teamcode.Tuners;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;

@Config
@TeleOp(name = "ArmMotorTuner", group = "Test")
public class intakeArmTuner extends BaseOpMode
{
    //IntakeRotatingArm armMotor;
    DcMotorEx armMotor;
    private PIDController controller;
    public static double p= 0.02, i = 0.06, d = 0.0013;
    public static double f = 0.0;
    public static int target = 1;
    private final double ticks_in_degree = 587.3/360;



    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        //armMotor = new IntakeRotatingArm(hardwareMap.get(DcMotorEx.class, "armMotor"), hardwareMap.get(VoltageSensor.class, "Control Hub"));
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
//        outtakeLeftServo.setDirection(Servo.Direction.REVERSE);
        double position = -38.2; // Initialize to midpoint

        waitForStart();

        while (opModeIsActive())
        {
            controller.setPID(p,i,d);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
            double power = pid + ff;
            power *= 0.9;
            armMotor.setPower(power);
            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();




            if (gamepad1.dpad_left)
            {
                position -= 0.1;
            }
            if (gamepad1.dpad_right)
            {
                position += 0.1;
            }

            if (gamepad1.a)
            {
                position = -8;
            }
            else if (gamepad1.b)
            {
                position = -20;
            }
        }
    }
}
