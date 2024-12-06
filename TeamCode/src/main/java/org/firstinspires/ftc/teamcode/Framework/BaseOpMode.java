package org.firstinspires.ftc.teamcode.Framework;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public abstract class BaseOpMode extends LinearOpMode {

    protected Drivetrain drivetrain;
    //four drivetrain motors
    protected DcMotorEx slideMotorLeft;
    protected DcMotorEx slideMotorRight;
    protected Slides slides;
    //two slide motors
    protected IntakeRotatingArm armMotor;
    //one intake rotating arm motor

    //slides class
    protected IntakeClaw intakeServo;
    protected OuttakeClaw outtakeServo;
    //two claw servos
    protected OuttakeRotatingArmServos outtakeRotatingArmServos;
    protected Slides slideController;
    private double p = 0, i = 0, d = 0;

    //two outtake arm servos
    protected IMU imu;

    protected void initHardware() {

        // wheel motors

        DcMotor[] driveMotors = {
                hardwareMap.dcMotor.get("motorFL"),
                hardwareMap.dcMotor.get("motorBL"),
                hardwareMap.dcMotor.get("motorFR"),
                hardwareMap.dcMotor.get("motorBR")};

        drivetrain = new Drivetrain(driveMotors);


        drivetrain.setMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.FORWARD, // motorFL
                DcMotorSimple.Direction.FORWARD, // motorBL
                DcMotorSimple.Direction.REVERSE, // motorFR
                DcMotorSimple.Direction.REVERSE  // motorBR
        });

        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        //intake claw servo
        intakeServo = new IntakeClaw(hardwareMap.get(Servo.class, "intakeServo"));
        imu = hardwareMap.get(IMU.class, "imu");

        outtakeServo = new OuttakeClaw(hardwareMap.get(Servo.class, "outtakeServo"));

        armMotor = new IntakeRotatingArm(hardwareMap.get(DcMotorEx.class, "armMotor"), hardwareMap.get(VoltageSensor.class, "Control Hub"));

        outtakeRotatingArmServos = new OuttakeRotatingArmServos(hardwareMap.get(Servo.class, "outtakeArmServoLeft"), hardwareMap.get(Servo.class, "outtakeArmServoRight"));

        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slides = new Slides(slideMotorLeft, slideMotorRight, voltageSensor);

    }


}


