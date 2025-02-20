package org.firstinspires.ftc.teamcode.Autonomous;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.Framework.Slides;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Config
@Autonomous(name = "jocelynsampleauto", group = "Autonomous")
public class jocelynsampleauto extends BaseOpMode {


    public boolean actionRunning = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.


        initHardware();
        Pose2d initialPose = new Pose2d(22, 61.2, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder hangSpecimen = drive.actionBuilder(initialPose)
                .lineToY(50)
                .strafeToLinearHeading(new Vector2d(5, 34.5), Math.toRadians(90));

        Pose2d specimenHangingPose = new Pose2d(5, 34.5, Math.toRadians(90));

        TrajectoryActionBuilder getRightMostSample = drive.actionBuilder(specimenHangingPose)
                .strafeToLinearHeading(new Vector2d(50.5, 47.5), Math.toRadians(270));

        Pose2d rightSampleGettingPose = new Pose2d(50.5, 47.5, Math.toRadians(270));

        TrajectoryActionBuilder approachBasket = drive.actionBuilder(rightSampleGettingPose)
                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(225));

        Pose2d approachBasketPose = new Pose2d(50, 53, Math.toRadians(225));

        TrajectoryActionBuilder dropSample = drive.actionBuilder(approachBasketPose)
                .strafeToLinearHeading(new Vector2d(54.5, 57.5), Math.toRadians(225));

        Pose2d dropSamplePose = new Pose2d(54.5, 57.5, Math.toRadians(225));

        TrajectoryActionBuilder backAway = drive.actionBuilder(dropSamplePose)
                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(225));

        Pose2d backAwayPose = new Pose2d(50, 53, Math.toRadians(225));

        TrajectoryActionBuilder getMiddleSample = drive.actionBuilder(backAwayPose)
                .strafeToLinearHeading(new Vector2d(50, 45), Math.toRadians(283));

        Pose2d middleSamplePose = new Pose2d(50, 45, Math.toRadians(283));

        TrajectoryActionBuilder approachBasketSecondTime = drive.actionBuilder(middleSamplePose)
                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(225));

        TrajectoryActionBuilder driveToAscent = drive.actionBuilder(backAwayPose)
                .splineToLinearHeading(new Pose2d(15, 10, Math.toRadians(225)), Math.toRadians(180));



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
                                                    telemetry.update();
                                                    return false;
                                                }
                                        ),
                                        (telemetryPacket) -> {
                                            slides.update(true, false, 0);
                                            return actionRunning;
                                        }
                                ),

                                new ParallelAction(
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
                                new SleepAction(0.2),
                                outtakeServo.clawClose(),
                                new SleepAction(0.2),

                                new ParallelAction(
                                        approachBasketSecondTime.build(),
                                        outtakeServo.clawClose(),
                                        outtakeRotatingArm.outtakeDeposit()
                                ),
                                 slides.slidesHighest(),
                                 (telemetryPacket) -> {
                                     actionRunning = true;
                                     telemetry.update();
                                     return false;
                                 },
                                 new ParallelAction(
                                         new SequentialAction(
                                                 dropSample.build(),
                                                 outtakeServo.clawOpen(),
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
                                 outtakeServo.clawClose(),
                                 outtakeRotatingArm.outtakeInit(),
                                 new ParallelAction(
                                        new SequentialAction(
                                                wrist.wristDown(),
                                                extendo.extendoRetractFull()
                                        ),
                                        new SequentialAction(
                                                backAway.build(),
                                                slides.slidesFullDown()
                                        )
                                 ),

                                 new ParallelAction(
                                         driveToAscent.build()
                                 )




                        )





        );

    }
}