package org.firstinspires.ftc.teamcode.Autonomous.Prototypes;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "JellyBotR", group = "Autonomous")
public class JellyBotR extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        Pose2d initialPose = new Pose2d(-23.5, 62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.pose = new Pose2d(-23.5, 62, Math.toRadians(90));
        Actions.runBlocking(slides.new SlidesDown());
        Actions.runBlocking(outtakeServo.new ClawClose());
        Actions.runBlocking(wrist.new WristDown());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(-50.2, 62), Math.toRadians(0))
                        .build()
        );
//        Actions.runBlocking(
//                new SequentialAction(
//                        outtakeRotatingArm.new OuttakeDeposit(),
//                        drive.actionBuilder(drive.pose)
//                                .splineToConstantHeading(new Vector2d(-5, 35.3), Math.toRadians(0))
//                                .build(),
//                        slides.new SlidesUp(),
//                        outtakeServo.new ClawOpen(),
//                        slides.new SlidesDown()
//                )
//        );
//        drive.updatePoseEstimate();
//        Actions.runBlocking(
//                new SequentialAction(
//                        drive.actionBuilder(drive.pose)
//                                .splineToLinearHeading(new Pose2d(-40.2, 50, Math.toRadians(270)), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(-48.2, 45.3), Math.toRadians(0))
//                                .build(),
//                        intakeServo.new ClawOpen(),
//                        extendo.new ExtendoExtend(),
//                        intakeServo.new ClawClose(),
//                        extendo.new ExtendoRetract(),
//                        slides.new SlidesTransfer(),
//                        outtakeRotatingArm.new OuttakeTransfer(),
//                        wrist.new WristUp(),
//                        outtakeServo.new ClawClose(),
//                        intakeServo.new ClawOpen(),
//                        wrist.new WristDown(),
//                        outtakeRotatingArm.new OuttakeDeposit()
//                )
//        );
//        drive.updatePoseEstimate();
//        Actions.runBlocking(
//                new SequentialAction(
//                        drive.actionBuilder(drive.pose)
//                                .splineToConstantHeading(new Vector2d(-48.2, 55.3), Math.toRadians(0))
//                                .build(),
//                        outtakeServo.new ClawOpen()
//                )
//        );
//        drive.updatePoseEstimate();
//        Actions.runBlocking(
//                new SequentialAction(
//                        drive.actionBuilder(drive.pose)
//                                .splineToConstantHeading(new Vector2d (-58.2, 45.3), Math.toRadians(0))
//                                .build(),
//                        extendo.new ExtendoExtend(),
//                        intakeServo.new ClawClose(),
//                        extendo.new ExtendoRetract(),
//                        slides.new SlidesTransfer(),
//                        outtakeRotatingArm.new OuttakeTransfer(),
//                        wrist.new WristUp(),
//                        outtakeServo.new ClawClose(),
//                        intakeServo.new ClawOpen(),
//                        wrist.new WristDown()
//                )
//        );
//        drive.updatePoseEstimate();
//        Actions.runBlocking(
//                new SequentialAction(
//                        drive.actionBuilder(drive.pose)
//                                .splineToConstantHeading(new Vector2d (-58.2, 61.3), Math.toRadians(0))
//                                .build()
//                )
//        );

    }
}
