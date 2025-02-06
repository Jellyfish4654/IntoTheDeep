package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
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
        Pose2d initialPose = new Pose2d(-23.5, 62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.pose = new Pose2d(-23.5, 62, Math.toRadians(90));
        Actions.runBlocking(slides.slidesDown());
        Actions.runBlocking(outtakeServo.clawClose());
        Actions.runBlocking(wrist.wristDown());

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(initialPose)
                .lineToY(42)
                .setTangent(Math.toRadians(0))
                .lineToX(-5)
                .setTangent(Math.toRadians(90))
                .lineToY(35.3);

        Pose2d secondPose = new Pose2d(-5, 35.3, Math.toRadians(90));
        TrajectoryActionBuilder toSample = drive.actionBuilder(secondPose)
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(-48.2)
                .setTangent(Math.toRadians(270));

        Pose2d thirdPose = new Pose2d(-48.2, 45.3, Math.toRadians(270));
        TrajectoryActionBuilder toObservationZone = drive.actionBuilder(thirdPose)
                .lineToY(55.3);

        Pose2d fourthPose = new Pose2d(-48.2, 55.3, Math.toRadians(270));
        TrajectoryActionBuilder toSample2 = drive.actionBuilder(fourthPose)
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(-58.2)
                .setTangent(Math.toRadians(270));

        Pose2d fifthPose = new Pose2d(-58.2, 45.3, Math.toRadians(270));
        TrajectoryActionBuilder toPark = drive.actionBuilder(fifthPose)
                .lineToY(61.3)
                .waitSeconds(2); // park

        waitForStart();
        if (isStopRequested()) return;
        Action action1 = toSubmersible.build();
        Action action2 = toSample.build();
        Action action3 = toObservationZone.build();
        Action action4 = toSample2.build();
        Action action5 = toPark.build();

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        outtakeRotatingArmServos.outtakeDeposit(),
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(-5, 35.3), Math.toRadians(0))
                                .build(),
                        slides.slidesUp(),
                        outtakeServo.clawOpen(),
                        slides.slidesDown()
                        )
        ));
        drive.updatePoseEstimate();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .splineToLinearHeading(new Pose2d(-40.2, 50, Math.toRadians(270)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-48.2, 45.3), Math.toRadians(0))
                                .build(),
                        intakeServo.clawOpen(),
                        extendo.extendoExtend(),
                        intakeServo.clawClose(),
                        extendo.extendoRetract(),
                        outtakeRotatingArmServos.outtakeTransfer(),
                        wrist.wristUp(),
                        outtakeServo.clawClose(),
                        intakeServo.clawOpen(),
                        wrist.wristDown(),
                        outtakeRotatingArmServos.outtakeDeposit()
                )
        ));
        drive.updatePoseEstimate();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(-48.2, 55.3), Math.toRadians(0))
                                .build(),
                        outtakeServo.clawOpen()
                )
        ));
        drive.updatePoseEstimate();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d (-58.2, 45.3), Math.toRadians(0))
                                .build(),
                        extendo.extendoExtend(),
                        intakeServo.clawClose(),
                        extendo.extendoRetract(),
                        outtakeRotatingArmServos.outtakeTransfer(),
                        wrist.wristUp(),
                        outtakeServo.clawClose(),
                        intakeServo.clawOpen(),
                        wrist.wristDown()
                )
        ));
        drive.updatePoseEstimate();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d (-58.2, 61.3), Math.toRadians(0))
                                .build()
                )
        ));

    }
}
