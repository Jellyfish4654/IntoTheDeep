package org.firstinspires.ftc.teamcode.Autonomous.Current;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.Framework.Hardware.Slides;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Config
@Autonomous(name = "jocelynsampleauto", group = "Autonomous")
public class jocelynsampleauto extends BaseOpMode {


    public boolean actionRunning = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.


        initHardware();
        Pose2d initialPose = new Pose2d(22, 60.1, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder hangSpecimen = drive.actionBuilder(initialPose)
                .lineToY(50)
                .strafeToLinearHeading(new Vector2d(5, 34), Math.toRadians(90));

        Pose2d specimenHangingPose = new Pose2d(5, 34, Math.toRadians(90));

        TrajectoryActionBuilder getRightMostSample = drive.actionBuilder(specimenHangingPose)
                .strafeToLinearHeading(new Vector2d(50.5, 47.5), Math.toRadians(270));

        Pose2d rightSampleGettingPose = new Pose2d(50.5, 47.5, Math.toRadians(270));

        TrajectoryActionBuilder approachBasket = drive.actionBuilder(rightSampleGettingPose)
                .strafeToLinearHeading(new Vector2d(52.5, 51), Math.toRadians(225));

        Pose2d approachBasketPose = new Pose2d(52.5, 51, Math.toRadians(225));

        TrajectoryActionBuilder dropSample = drive.actionBuilder(approachBasketPose)
                .strafeToLinearHeading(new Vector2d(52, 58), Math.toRadians(225));

        Pose2d dropSamplePose = new Pose2d(52, 58, Math.toRadians(225));

        TrajectoryActionBuilder backAway = drive.actionBuilder(dropSamplePose)
                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(225));

        Pose2d backAwayPose = new Pose2d(50, 53, Math.toRadians(225));

        TrajectoryActionBuilder getMiddleSample = drive.actionBuilder(backAwayPose)
                .strafeToLinearHeading(new Vector2d(57, 47), Math.toRadians(283));

        Pose2d middleSamplePose = new Pose2d(57, 47, Math.toRadians(283));

        TrajectoryActionBuilder approachBasketSecondTime = drive.actionBuilder(middleSamplePose)
                .strafeToLinearHeading(new Vector2d(52.5, 51), Math.toRadians(225));

        TrajectoryActionBuilder driveToAscent = drive.actionBuilder(backAwayPose)
                .splineToLinearHeading(new Pose2d(22, 10, Math.toRadians(45)), Math.toRadians(180));



        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("left target", Slides.leftTarget);
            telemetry.addData("left slide pos", slides.getCurrentLeftPosition());
            telemetry.addData("action running", actionRunning);
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

                        getRightMostSample.build(),

                        new ParallelAction(
                                extendo.extendoExtend(),
                                wrist.wristDown()
                        ),

                        //extend to reach sample
                        new SleepAction(0.1),

                        intakeServo.clawClose(),
                        new SleepAction(0.1),

                        //grab sample

                        new ParallelAction(
                                wrist.wristUp(),
                                extendo.extendoRetract()
                        ),

                        // transfer sample

                        intakeServo.clawOpen(),
                        new ParallelAction(
                                slides.slidesTransfer(),
                                outtakeRotatingArm.outtakeTransfer()
                        ),


                        new SleepAction(0.2),
                        outtakeServo.clawClose(),
                        new SleepAction(0.2),

                        new ParallelAction(
                                approachBasket.build(),
                                slides.slidesHighest(),
                                outtakeServo.clawClose(),
                                outtakeRotatingArm.outtakeDeposit()
                        ),

                        //drop sample
                        slides.slidesHighest(),

                        new ParallelAction(
                                new SequentialAction(
                                        dropSample.build(),
                                        outtakeServo.clawOpen(),
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
                ));
        actionRunning = true;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                outtakeServo.clawOpen(),
                                backAway.build(),
                                outtakeRotatingArm.outtakeTransfer(),
                                slides.slidesTransfer()
                        ),

                        getMiddleSample.build(),

                        new ParallelAction(
                                extendo.extendoExtend(),
                                wrist.wristDown()
                        ),
                        new SleepAction(0.3),

                        intakeServo.clawClose(),
                        new SleepAction(0.2),

                        new ParallelAction(
                                wrist.wristUp(),
                                extendo.extendoRetract()
                        ),


                        intakeServo.clawOpen(),
                        new ParallelAction(
                                slides.slidesTransfer(),
                                outtakeRotatingArm.outtakeTransfer()
                        ),

                        new ParallelAction(
                                approachBasketSecondTime.build(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        outtakeServo.clawClose(),
                                        new SleepAction(0.2),
                                        outtakeRotatingArm.outtakeDeposit()
                                )
                        ),
                        slides.slidesHighest(),

                        new ParallelAction(
                                new SequentialAction(
                                        dropSample.build(),
                                        outtakeServo.clawOpen(),
                                        new SleepAction(0.2),
                                        (telemetryPacket) -> {
                                            actionRunning = false;
                                            telemetry.update();
                                            return false;
                                        }
                                ),
                                (telemetryPacket) -> {
                                    slides.update(true, false, 0);
                                    return actionRunning;
                                }
                        ),
                        outtakeRotatingArm.outtakeInit(),
                        new ParallelAction(
                                extendo.extendoRetract(),
                                new SequentialAction(
                                        backAway.build(),
                                        slides.slidesFullDown()
                                )
                        ),
                        driveToAscent.build()

                )

        );
    }
}
