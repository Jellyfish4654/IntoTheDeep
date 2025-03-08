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

@Config
@Autonomous(name = "martin", group = "Autonomous")
public class brokenMartinSampleAuto extends BaseOpMode {
    public boolean actionRunning = true;
    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        initHardware();
        double scale = 0.6;

        Pose2d initialPose = new Pose2d(22*scale, 60.1*scale, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder hangSpecimen = drive.actionBuilder(initialPose)
                .lineToY(50*scale)
                .strafeToLinearHeading(new Vector2d(0*scale, 33*scale), Math.toRadians(90));


        Pose2d specimenHangingPose = new Pose2d(0*scale, 33*scale, Math.toRadians(90));


        TrajectoryActionBuilder getRightMostSample = drive.actionBuilder(specimenHangingPose)
                .strafeToLinearHeading(new Vector2d(48.4*scale, 45*scale), Math.toRadians(270));


        Pose2d rightSampleGettingPose = new Pose2d(48.4*scale, 45*scale, Math.toRadians(270));


        TrajectoryActionBuilder approachBasket = drive.actionBuilder(rightSampleGettingPose)
                .strafeToLinearHeading(new Vector2d(52.5*scale, 51.5*scale), Math.toRadians(225));


        Pose2d approachBasketPose = new Pose2d(52.5*scale, 51.5*scale, Math.toRadians(225));


        TrajectoryActionBuilder backAway = drive.actionBuilder(approachBasketPose)
                .strafeToLinearHeading(new Vector2d(50*scale, 53*scale), Math.toRadians(225));

        TrajectoryActionBuilder backAwayTwo = drive.actionBuilder(approachBasketPose)
                .strafeToLinearHeading(new Vector2d(50*scale, 53*scale), Math.toRadians(225));


        Pose2d backAwayPose = new Pose2d(50*scale, 53*scale, Math.toRadians(225));

        TrajectoryActionBuilder getMiddleSample = drive.actionBuilder(backAwayPose)
                .strafeToLinearHeading(new Vector2d(56*scale, 45.5*scale), Math.toRadians(275.5));


        Pose2d middleSamplePose = new Pose2d(56*scale, 45.5*scale, Math.toRadians(275.5));

        TrajectoryActionBuilder approachBasketSecondTime = drive.actionBuilder(middleSamplePose)
                .strafeToLinearHeading(new Vector2d(53*scale, 51.5*scale), Math.toRadians(225));

        Pose2d approachBasketSecondTimePose = new Pose2d(53*scale, 51.5*scale, Math.toRadians(225));

        TrajectoryActionBuilder getLeftSample = drive.actionBuilder(approachBasketSecondTimePose)
                .strafeToLinearHeading(new Vector2d(62.25*scale, 43.7*scale), Math.toRadians(292));

        Pose2d getLeftSamplePose = new Pose2d(62.25*scale, 43.7*scale, Math.toRadians(292));

        TrajectoryActionBuilder approachBasketThirdTime = drive.actionBuilder(getLeftSamplePose)
                .strafeToLinearHeading(new Vector2d(53*scale, 51.5*scale), Math.toRadians(220));

        Pose2d approachBasketThirdTimePose = new Pose2d(53*scale, 51.5*scale, Math.toRadians(220));

        TrajectoryActionBuilder backAwayThree = drive.actionBuilder(approachBasketThirdTimePose)
                .strafeToLinearHeading(new Vector2d(40*scale, 45*scale), Math.toRadians(220));




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("left target", Slides.leftTarget);
            telemetry.addData("left slide pos", slides.getCurrentLeftPosition());
            telemetry.addData("action running", actionRunning);
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
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
                                new SleepAction(0.3),
                                intakeServo.clawClose(),
                                new SleepAction(0.3),
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
                                new SleepAction(0.5),
                                outtakeServo.clawClose(),
                                new SleepAction(0.4),


                                new ParallelAction(
                                        approachBasket.build(),
                                        slides.slidesHighest(),
                                        outtakeRotatingArm.outtakeDeposit()
                                ),
                                //drop sample
                                outtakeServo.clawOpen(),
                                new SleepAction(0.8),
                                new ParallelAction(
                                        backAway.build(),
                                        outtakeRotatingArm.outtakeTransfer()
                                ),
                                new ParallelAction(
                                        slides.slidesTransfer(),
                                        getMiddleSample.build()
                                ),
                                extendo.extendoExtend(),
                                wrist.wristDown(),
                                new SleepAction(0.8),
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
                                new SleepAction(0.7),
                                outtakeServo.clawClose(),
                                new SleepAction(0.6),
                                new ParallelAction(
                                        approachBasketSecondTime.build(),
                                        slides.slidesHighest(),
                                        outtakeRotatingArm.outtakeDeposit()
                                ),
                                outtakeServo.clawOpen(),
                                new SleepAction(0.8),
                                outtakeRotatingArm.outtakeInit(),
                                backAwayTwo.build(),
                                new ParallelAction(
                                        slides.slidesTransfer(),
                                        outtakeRotatingArm.outtakeTransfer(),
                                        getLeftSample.build()
                                ),



                                //third sample
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
                                new SleepAction(0.5),
                                intakeServo.clawOpen(),
                                new ParallelAction(
                                        slides.slidesTransfer(),
                                        outtakeRotatingArm.outtakeTransfer()
                                ),
                                new SleepAction(0.8),
                                outtakeServo.clawClose(),
                                new SleepAction(0.4),
                                new ParallelAction(
                                        approachBasketThirdTime.build(),
                                        slides.slidesHighest(),
                                        outtakeRotatingArm.outtakeDeposit()
                                ),
                                outtakeServo.clawOpen(),
                                new SleepAction(0.8),
                                outtakeRotatingArm.outtakeTransfer(),
                                new ParallelAction(
                                        extendo.extendoRetract(),
                                        new SleepAction(0.2),
                                        new ParallelAction(
                                                backAwayThree.build(),
                                                slides.slidesFullDown(),
                                                outtakeRotatingArm.outtakeDeposit()
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