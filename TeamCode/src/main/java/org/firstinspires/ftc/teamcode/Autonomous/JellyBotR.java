package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "JellyBotR", group = "Autonomous")
public class JellyBotR extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-23.5, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Actions.runBlocking(slides.slidesDown());
        TrajectoryActionBuilder movement = drive.actionBuilder(initialPose)
                .lineToY(42)
                .setTangent(Math.toRadians(0))
                .lineToX(-5)
                .setTangent(Math.toRadians(90))
                .lineToY(35.3)
                .waitSeconds(3) // hang specimen
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(-48.2)
                .setTangent(Math.toRadians(270))
                .waitSeconds(2) // grab sample
                .lineToY(55.3)
                .waitSeconds(2) // drop sample
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(-58.2)
                .setTangent(Math.toRadians(270))
                .waitSeconds(2) // grab sample
                .lineToY(61.3)
                .waitSeconds(2); // park
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Action trajectoryActionChosen = movement.build();
        Actions.runBlocking(
                new SequentialAction(
                        slides.slidesUp(),
                        intakeServo.clawOpen(),
                        outtakeServo.clawClose(),
                        extendo.extendoExtend(),
                        wrist.wristUp(),
                        trajectoryActionChosen
                )
        );
    }
}
