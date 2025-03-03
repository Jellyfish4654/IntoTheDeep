package org.firstinspires.ftc.teamcode.Autonomous.Prototypes;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "JellyBotL", group = "Autonomous")
public class JellyBotL extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        Pose2d initialPose = new Pose2d(23.5, 62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.pose = new Pose2d(23.5, 62, Math.toRadians(90));
        Actions.runBlocking(slides.new SlidesDown());
        Actions.runBlocking(outtakeServo.new ClawClose());
        Actions.runBlocking(wrist.new WristDown());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(23.5, 42, Math.toRadians(0)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(5, 35.3, Math.toRadians(90)), Math.toRadians(0))
                        .build(),
                outtakeRotatingArm.new OuttakeDeposit(),
                slides.new SlidesUp(),
                outtakeServo.new ClawOpen()
                )
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                slides.new SlidesDown(),
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(48.2, 45.3, Math.toRadians(90)), Math.toRadians(0))
                        .build(),
                extendo.new ExtendoExtend(),
                intakeServo.new ClawClose(),
                extendo.new ExtendoRetract(),
                slides.new SlidesTransfer(),
                outtakeRotatingArm.new OuttakeTransfer(),
                wrist.new WristUp(),
                outtakeServo.new ClawClose(),
                intakeServo.new ClawOpen()
                )

        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(53.2, 51.8, Math.toRadians(45)), Math.toRadians(0))
                        .build(),
                slides.new SlidesHighest(),
                outtakeRotatingArm.new OuttakeDeposit(),
                outtakeServo.new ClawOpen(),
                outtakeRotatingArm.new OuttakeTransfer(),
                slides.new SlidesDown()
                )
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(-40.2, 50, Math.toRadians(90)), Math.toRadians(0))
                        .build(),
                wrist.new WristDown(),
                extendo.new ExtendoExtend(),
                intakeServo.new ClawClose(),
                extendo.new ExtendoRetract(),
                slides.new SlidesTransfer(),
                wrist.new WristUp(),
                outtakeServo.new ClawClose(),
                intakeServo.new ClawOpen()
                )

        );
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(51.8, 53.2, Math.toRadians(45)), Math.toRadians(0))
                        .build(),
                slides.new SlidesHighest(),
                outtakeRotatingArm.new OuttakeDeposit(),
                outtakeServo.new ClawOpen(),
                slides.new SlidesDown()
                )

        );
    }
}
