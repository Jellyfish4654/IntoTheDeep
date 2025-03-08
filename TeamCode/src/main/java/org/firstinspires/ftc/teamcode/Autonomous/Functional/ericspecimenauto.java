package org.firstinspires.ftc.teamcode.Autonomous.Functional;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.Framework.Hardware.Slides;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Config
@Autonomous(name = "ericspecimenauto", group = "Autonomous")
public class ericspecimenauto extends BaseOpMode {

    public boolean actionRunning = true;
    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        initHardware(true);
        Pose2d initialPose = new Pose2d(-22, 61.2, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder hangSpecimen = drive.actionBuilder(initialPose)
                .lineToY(50)
                .strafeToLinearHeading(new Vector2d(-5, 35), Math.toRadians(90));

        Pose2d specimenHangingPose = new Pose2d(-5, 35, Math.toRadians(90));

        TrajectoryActionBuilder pushFirstSample = drive.actionBuilder(specimenHangingPose)
//                .strafeToConstantHeading(new Vector2d(-29, 35))
//                .strafeToConstantHeading(new Vector2d(-44, 10))
                .strafeToConstantHeading(new Vector2d(-15, 40))

                .splineToLinearHeading(new Pose2d(-30, 5, Math.toRadians(85)), Math.toRadians(0))
                .lineToX(-42.5)
                .strafeToConstantHeading(new Vector2d(-42.5, 52.5));

        Pose2d pushSamplePose = new Pose2d(-43, 52.5, Math.toRadians(85));

//        TrajectoryActionBuilder pushSecondSample = drive.actionBuilder(pushSamplePose)
//                .strafeToConstantHeading(new Vector2d(-35, 45))
//                .strafeToConstantHeading(new Vector2d(-35, 10))
//                .strafeToConstantHeading(new Vector2d(-54, 10))
//                .strafeToConstantHeading(new Vector2d(-54, 57));

        Pose2d pushSamplePoseTwo = new Pose2d(-54, 57, Math.toRadians(85));

        TrajectoryActionBuilder readyGrabSpecimen = drive.actionBuilder(pushSamplePose)
                .strafeToConstantHeading(new Vector2d(-40, 53))
                .turn(Math.toRadians(180));

        Pose2d readyGrabSpecimenPose = new Pose2d(-42, 53, Math.toRadians(268));

        TrajectoryActionBuilder grabSpecimen = drive.actionBuilder(readyGrabSpecimenPose)
                .strafeToConstantHeading(new Vector2d(-45, 56.5));

        Pose2d grabSpecimenPose = new Pose2d(-45, 56.5, Math.toRadians(268));

        TrajectoryActionBuilder hangSecondSpecimen = drive.actionBuilder(grabSpecimenPose)
                .strafeToLinearHeading(new Vector2d(5, 33), Math.toRadians(90));

        Pose2d hangedSecondSpecimenPose = new Pose2d(5, 33, Math.toRadians(90));

        TrajectoryActionBuilder grabThirdSpecimen = drive.actionBuilder(hangedSecondSpecimenPose)
                .strafeToConstantHeading(new Vector2d(-43, 53))
                .turn(Math.toRadians(170))
                .strafeToConstantHeading(new Vector2d(-43, 58));

        Pose2d grabThirdSpecimenPose = new Pose2d(-43, 58, Math.toRadians(263.5));

        TrajectoryActionBuilder hangThirdSpecimen = drive.actionBuilder(grabThirdSpecimenPose)
                .strafeToLinearHeading(new Vector2d(-2, 31), Math.toRadians(84.25));




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("left target", Slides.leftTarget);
            telemetry.addData("left slide pos", slides.getCurrentLeftPosition());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(

                        outtakeServo.clawClose(),
                        new ParallelAction(
                                hangSpecimen.build(),
                                new SequentialAction(
                                        outtakeRotatingArm.outtakeChamber(),
                                        new SleepAction(0.55),
                                        slides.slidesUnderBar()
                                )
                        ),

                        //go to specimen bar

                        new ParallelAction(
                                slides.slidesOverBar(),
                                new SequentialAction(
                                        new SleepAction(0.8),
                                        outtakeServo.clawOpen()
                                )
                        ),

                        // hang specimen

                        pushFirstSample.build(),
//                        pushSecondSample.build(),


                        new ParallelAction(
                                new SequentialAction(
                                        new ParallelAction(
                                                slides.slidesDown(),
                                                outtakeRotatingArm.outtakeGrabSpecimen(),
                                                outtakeServo.clawOpen(),
                                                readyGrabSpecimen.build()
                                        ),
                                        new SequentialAction(
                                                grabSpecimen.build(),
                                                new SleepAction(0.1),
                                                outtakeServo.clawClose(),
                                                new SleepAction(0.2),
                                                slides.slidesUp(),
                                                hangSecondSpecimen.build(),
                                                new ParallelAction(
                                                        slides.slidesTransfer(),
                                                        new SequentialAction(
                                                                new SleepAction(0.35),
                                                                outtakeServo.clawOpen()
                                                        )
                                                ),
                                                (telemetryPacket) -> {
                                                    actionRunning = false;
                                                    return false;
                                                }
                                        )
                                ),
                                (telemetryPacket) -> {
                                    slides.update(true, false, 0);
                                    return actionRunning;
                                }
                        )

                )
        );
        actionRunning = true;
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        slides.slidesDown(),
                                        outtakeRotatingArm.outtakeGrabSpecimen(),
                                        outtakeServo.clawOpen(),
                                        grabThirdSpecimen.build()
                                ),
                                new SleepAction(0.2),
                                outtakeServo.clawClose(),
                                new SleepAction(0.3),
                                slides.slidesUnderBar(),
                                outtakeRotatingArm.outtakeChamber(),
                                slides.slidesUp(),
                                hangThirdSpecimen.build(),
                                new ParallelAction(
                                        slides.slidesTransfer(),
                                        new SequentialAction(
                                                new SleepAction(0.35),
                                                outtakeServo.clawOpen()
                                        )
                                ),
                                (telemetryPacket) -> {
                                    actionRunning = false;
                                    return false;
                                }

                        ),
                        (telemetryPacket) -> {
                            slides.update(true, false, 0);
                            return actionRunning;
                        }

                )
        );

    }
}