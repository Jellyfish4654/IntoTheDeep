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
@Autonomous(name = "jocelynsampleauto", group = "Autonomous")
public class jocelynsampleauto extends BaseOpMode {
    public boolean actionRunning = true;
    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        initHardware(true);
        Pose2d initialPose = new Pose2d(20.5, 60.1, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder approachBasketFirst = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(52.5, 52.5), Math.toRadians(225));


        Pose2d approachBasketPoseFirst = new Pose2d(52.5, 52.5, Math.toRadians(225));


        TrajectoryActionBuilder getRightMostSample = drive.actionBuilder(approachBasketPoseFirst)
                .strafeToLinearHeading(new Vector2d(44, 45), Math.toRadians(266));


        Pose2d rightSampleGettingPose = new Pose2d(44, 45, Math.toRadians(266));


        TrajectoryActionBuilder approachBasket = drive.actionBuilder(rightSampleGettingPose)
                .strafeToLinearHeading(new Vector2d(52.5, 51.5), Math.toRadians(222));


        Pose2d approachBasketPose = new Pose2d(52.5, 51.5, Math.toRadians(222));


        TrajectoryActionBuilder backAway = drive.actionBuilder(approachBasketPose)
                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(225));

        TrajectoryActionBuilder backAwayTwo = drive.actionBuilder(approachBasketPose)
                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(225));


        Pose2d backAwayPose = new Pose2d(50, 53, Math.toRadians(225));

        TrajectoryActionBuilder getMiddleSample = drive.actionBuilder(backAwayPose)
                .strafeToLinearHeading(new Vector2d(58.5, 45.5), Math.toRadians(274));


        Pose2d middleSamplePose = new Pose2d(58.5, 45.5, Math.toRadians(274));

        TrajectoryActionBuilder approachBasketSecondTime = drive.actionBuilder(middleSamplePose)
                .strafeToLinearHeading(new Vector2d(53, 51.5), Math.toRadians(225));

        Pose2d approachBasketSecondTimePose = new Pose2d(53, 51.5, Math.toRadians(225));

        TrajectoryActionBuilder getLeftSample = drive.actionBuilder(approachBasketSecondTimePose)
                .strafeToLinearHeading(new Vector2d(63.25, 43.7), Math.toRadians(296));

        Pose2d getLeftSamplePose = new Pose2d(63.25, 43.7, Math.toRadians(296));

        TrajectoryActionBuilder approachBasketThirdTime = drive.actionBuilder(getLeftSamplePose)
                .strafeToLinearHeading(new Vector2d(53.5, 52), Math.toRadians(220));

        Pose2d approachBasketThirdTimePose = new Pose2d(53.5, 52, Math.toRadians(220));

        TrajectoryActionBuilder backAwayThree = drive.actionBuilder(approachBasketThirdTimePose)
                .strafeToLinearHeading(new Vector2d(40, 45), Math.toRadians(220));




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
                                        approachBasketFirst.build(),
                                        slides.slidesHighest(),
                                        outtakeRotatingArm.outtakeDeposit()
                                ),
                                outtakeServo.clawOpen(),

                                //drop sample
                                new SleepAction(0.8),
                                new ParallelAction(
                                        backAway.build(),
                                        outtakeRotatingArm.outtakeTransfer()
                                ),
                                new ParallelAction(
                                        slides.slidesTransfer(),
                                        getRightMostSample.build()
                                ),


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