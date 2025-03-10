package org.firstinspires.ftc.teamcode.Framework.Old;

// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeRotatingArm {
    private DcMotorEx armMotor;
    private VoltageSensor voltageSensor;
    private ElapsedTime timer;
    private double kcos = 0.25;
    private final double ticks_in_degrees = 537.7 / 360.0;
    //private double kP = 0.00, kI = 0, kD = 0;
    //	private double kPLeft = 0.01, kILeft = 0, kDLeft = 0.0002, kFLeft = 0.01;
//	private double kPRight = 0.01, kIRight = 0, kDRight = 0.0002, kFRight = 0.01;
    private double lastError = 0;
    private double integralSum = 0;
    //private double kG = 1.5;
    private double targetPosition;
    private double voltageCompensation;
    public static double p= 0.02, i = 0.06, d = 0.0013;
    public static double f = 0.0;
    public static int target = -177;
    private final double ticks_in_degree = 587.3/360;
    private PIDController intakeController;


    double position = -177; // Initialize to midpoint

    public IntakeRotatingArm(DcMotorEx armMotor, VoltageSensor voltageSensor) {
        this.armMotor = armMotor;
        this.voltageSensor = voltageSensor;
        this.timer = new ElapsedTime();
        setTargetPosition(position);
    }

    public void setTargetPosition(double TargetPosition) {
        targetPosition=TargetPosition;
        timer.reset();
    }

    public void update() {
        double elapsedTime = timer.seconds();
        control();
    }


    public void control() {
        double rightPower = calculateMotorPower(armMotor, targetPosition, intakeController);
        armMotor.setPower(rightPower);
    }
    public void manualIntake(double intakeJoystick) {
        armMotor.setPower(intakeJoystick);
    }

    private double calculateMotorPower(DcMotorEx motor, double targetPosition, PIDController intakeController) {
        timer.reset();
        intakeController = new PIDController(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = intakeController.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        double power = pid + ff;
        armMotor.setPower(power);
        return power;
    }
    public double getTargetPosition()
    {
        return targetPosition;
    }

    public int getCurrentPosition(){
        return armMotor.getCurrentPosition();
    }


}