package org.firstinspires.ftc.teamcode.Framework;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake 
{
    private DcMotorEx armMotor;
    
    private boolean initialized = false;
    public Intake(DcMotorEx armMotor) {
        this.armMotor = armMotor;
    }

    public void update() {
        control(targetPosition);
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket)

    if (!initialized) {
	setTargetPosition(360);
	initialized = true;

        start() {
            if (Gamepad1.b.isPressed) {
                int desiredPosition = 1000
                armMotor.setTargetPosition (desiredPosition);
                armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
        
            if (!currentGamepad1.b && previousGamepad1.b) {
                armMotor.setPosition(armMotor.getPosition() - 0.1);
                }

    
            }    
    }
}

