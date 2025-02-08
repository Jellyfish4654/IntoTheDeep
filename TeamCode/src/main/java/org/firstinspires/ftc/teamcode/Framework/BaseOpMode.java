package org.firstinspires.ftc.teamcode.Framework;

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
    protected DcMotorEx Extendo;
    protected Extendo extendo;
    protected Slides slides;
    //two slide motors
    //one intake rotating arm motor

    //slides class
    protected IntakeClaw intakeServo;
    protected OuttakeClaw outtakeServo;

    //two claw servos

    protected Wrist wrist;
    protected OuttakeRotatingArm outtakeRotatingArm;
    protected IntakeRotatingArmServos intakeRotatingArmServos;

    //two outtake arm servos
    protected IMU imu;

    public void initHardware() {

        // wheel motors

        DcMotor[] driveMotors = {
                hardwareMap.dcMotor.get("motorFL"),
                hardwareMap.dcMotor.get("motorBL"),
                hardwareMap.dcMotor.get("motorFR"),
                hardwareMap.dcMotor.get("motorBR")};

        drivetrain = new Drivetrain(driveMotors);


        drivetrain.setMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.FORWARD, // motorFL
                DcMotorSimple.Direction.REVERSE, // motorBL
                DcMotorSimple.Direction.REVERSE, // motorFR
                DcMotorSimple.Direction.REVERSE  // motorBR
        });

        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        //intake claw servo
        intakeServo = new IntakeClaw(hardwareMap.get(Servo.class, "intakeServo"));
        imu = hardwareMap.get(IMU.class, "imu");

        outtakeServo = new OuttakeClaw(hardwareMap.get(Servo.class, "outtakeServo"));
        intakeServo.openClaw();
        outtakeServo.closeClaw();
        wrist = new Wrist(hardwareMap.get(Servo.class, "wristServo"));

        outtakeRotatingArm = new OuttakeRotatingArm(hardwareMap.get(Servo.class, "outtakeArmServoLeft"));
        outtakeRotatingArm.armOuttakeDeposit();

        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        slides = new Slides(slideMotorLeft, slideMotorRight, voltageSensor);

        Extendo = hardwareMap.get(DcMotorEx.class, "extendo");

        extendo = new Extendo(Extendo, voltageSensor);
    }


}


