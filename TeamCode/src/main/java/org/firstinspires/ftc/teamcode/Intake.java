package org.firstinspires.ftc.teamcode;

public class Intake 
{
    private DcMotor armMotor;
    armMotor = hardwareMap.get(DcMotor.class, "Arm Motor");
    
    private boolean initialized = false;
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

