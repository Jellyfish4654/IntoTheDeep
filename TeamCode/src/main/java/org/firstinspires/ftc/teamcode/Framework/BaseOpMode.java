package org.firstinspires.ftc.teamcode.Framework;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public abstract class BaseOpMode extends LinearOpMode {

    protected DcMotor[] driveMotors;
    //protected Intake intakeSystem;
    protected DcMotorEx slideMotorLeft;
    protected DcMotorEx slideMotorRight;
    protected DcMotorEx armMotor;
    protected Slides slides;
    protected CRServo outtakeCRServo;
    protected IMU imuSensor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    GamepadEx thegamepad;

    protected DistanceSensor distanceLeft;

    protected DistanceSensor distanceRight;

    protected void initHardware() {

        // wheel motors

        driveMotors = new DcMotor[]{
                //control hub
                hardwareMap.dcMotor.get("motorFL"),
                hardwareMap.dcMotor.get("motorBL"),
                hardwareMap.dcMotor.get("motorFR"),
                hardwareMap.dcMotor.get("motorBR")

        };


        setMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.FORWARD, // motorFL
                DcMotorSimple.Direction.FORWARD, // motorBL
                DcMotorSimple.Direction.REVERSE, // motorFR
                DcMotorSimple.Direction.REVERSE  // motorBR
        });

        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        //intake claw servo
        Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setDirection(Servo.Direction.REVERSE);

        Servo outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");
        intakeServo.setDirection(Servo.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slides = new Slides(slideMotorLeft, slideMotorRight, voltageSensor);

        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

    }

    private void setMotorDirections(DcMotorSimple.Direction[] directions)
    {
        for (int i = 0; i < driveMotors.length; i++)
        {
            driveMotors[i].setDirection(directions[i]);
        }
    }

}
