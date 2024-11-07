package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BaseOpMode extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor;
        DcMotor backLeftMotor;
        DcMotor frontRightMotor;
        DcMotor backRightMotor;

        frontLeftMotor = hardwareMap.get(DcMotor .class, "motorFL");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motorBL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motorFR");
        backRightMotor = hardwareMap.get(DcMotor.class, "motorBR");
        DcMotorEx slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        DcMotorEx slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
    }

}
