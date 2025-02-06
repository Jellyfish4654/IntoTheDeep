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
@Autonomous(name = "JellyBotL", group = "Autonomous")
public class JellyBotL extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(23.5, 62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.pose = new Pose2d(23.5, 62, Math.toRadians(90));
        Actions.runBlocking(slides.slidesDown());
        Actions.runBlocking(outtakeServo.clawClose());
        Actions.runBlocking(wrist.wristDown());

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(initialPose)
                .lineToY(42)
                .setTangent(Math.toRadians(0))
                .lineToX(5)
                .setTangent(Math.toRadians(270))
                .lineToY(35.3);

        Pose2d secondPose = new Pose2d(5, 35.3, Math.toRadians(270));
        TrajectoryActionBuilder toSample = drive.actionBuilder(secondPose)
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(48.2)
                .setTangent(Math.toRadians(90));

        Pose2d thirdPose = new Pose2d(48.2, 45.3, Math.toRadians(90));
        TrajectoryActionBuilder toBasket = drive.actionBuilder(thirdPose)
                .lineToY(53.2)
                .setTangent(Math.toRadians(0))
                .lineToX(51.8)
                .setTangent(Math.toRadians(45));

        Pose2d fourthPose = new Pose2d(51.8, 53.2, Math.toRadians(45));
        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(fourthPose)
                .setTangent(Math.toRadians(0))
                .lineToX(48.2)
                .setTangent(Math.toRadians(90))
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(58.2)
                .setTangent(Math.toRadians(90));

        Pose2d fifthPose = new Pose2d(58.2, 45.3, Math.toRadians(90));
        TrajectoryActionBuilder toSecondBasket = drive.actionBuilder(fifthPose)
                .setTangent(Math.toRadians(0))
                .lineToX(51.8)
                .setTangent(Math.toRadians(270))
                .lineToY(53.2)
                .setTangent(Math.toRadians(45));


        waitForStart();
        if (isStopRequested()) return;
        Action action1 = toSubmersible.build();
        Action action2 = toSample.build();
        Action action3 = toBasket.build();
        Action action4 = toSecondSample.build();
        Action action5 = toSecondBasket.build();

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(23.5, 42, Math.toRadians(0)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(5, 35.3, Math.toRadians(90)), Math.toRadians(0))
                        .build(),
                outtakeRotatingArm.outtakeDeposit(),
                slides.slidesUp(),
                outtakeServo.clawOpen()
                )

        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                slides.slidesDown(),
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(48.2, 45.3, Math.toRadians(90)), Math.toRadians(0))
                        .build(),
                extendo.extendoExtend(),
                intakeServo.clawClose(),
                extendo.extendoRetract(),
                slides.slidesTransfer(),
                outtakeRotatingArm.outtakeTransfer(),
                wrist.wristUp(),
                outtakeServo.clawClose(),
                intakeServo.clawOpen()
                )

        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(53.2, 51.8, Math.toRadians(45)), Math.toRadians(0))
                        .build(),
                slides.slidesHighest(),
                outtakeRotatingArm.outtakeDeposit(),
                outtakeServo.clawOpen(),
                outtakeRotatingArm.outtakeTransfer(),
                slides.slidesDown()
                )

        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(-40.2, 50, Math.toRadians(90)), Math.toRadians(0))
                        .build(),
                wrist.wristDown(),
                extendo.extendoExtend(),
                intakeServo.clawClose(),
                extendo.extendoRetract(),
                slides.slidesTransfer(),
                wrist.wristUp(),
                outtakeServo.clawClose(),
                intakeServo.clawOpen()
                )

        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(51.8, 53.2, Math.toRadians(45)), Math.toRadians(0))
                        .build(),
                slides.slidesHighest(),
                outtakeRotatingArm.outtakeDeposit(),
                outtakeServo.clawOpen(),
                slides.slidesDown()
                )

        );


    }
}
