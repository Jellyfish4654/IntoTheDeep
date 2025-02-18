package org.firstinspires.ftc.teamcode.Autonomous;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        initHardware();
        Pose2d initialPose = new Pose2d(22, 61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder hangSpecimen = drive.actionBuilder(initialPose)
                .lineToY(50)
                .strafeToLinearHeading(new Vector2d(5, 34), Math.toRadians(90));

        Pose2d specimenHangingPose = new Pose2d(5, 34, Math.toRadians(90));

        TrajectoryActionBuilder getRightMostSample = drive.actionBuilder(specimenHangingPose)
                .strafeToLinearHeading(new Vector2d(45, 50), Math.toRadians(270));

        Pose2d sampleGettingPose = new Pose2d(45, 50, Math.toRadians(270));

        TrajectoryActionBuilder approachBasket = drive.actionBuilder(sampleGettingPose)
                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(315));

        Pose2d approachBasketPose = new Pose2d(50, 53, Math.toRadians(315));

        TrajectoryActionBuilder dropSample = drive.actionBuilder(approachBasketPose)
                .strafeToLinearHeading(new Vector2d(55, 56), Math.toRadians(315));

        Pose2d dropSamplePose = new Pose2d(55, 56, Math.toRadians(315));

        TrajectoryActionBuilder getMiddleSample = drive.actionBuilder(dropSamplePose)
                .strafeToLinearHeading(new Vector2d(50, 50), Math.toRadians(270));





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
                                        slides.slidesOverBar()
                                )
                        ),
                        slides.slidesUnderBar(),
                        outtakeServo.clawOpen(),
                        new ParallelAction(
                                getRightMostSample.build(),
                                slides.slidesTransfer(),
                                outtakeRotatingArm.outtakeTransfer()
                        ),

                        new ParallelAction(
                                extendo.extendoExtend(),
                                wrist.wristDown()
                        ),

                        intakeServo.clawClose(),
                        new ParallelAction(
                                wrist.wristUp(),
                                extendo.extendoRetract()
                        ),


                        intakeServo.clawOpen(),
                        new SleepAction(0.5),
                        outtakeServo.clawClose(),
                        approachBasket.build(),
                        new ParallelAction(
                                slides.slidesHighest(),
                                outtakeRotatingArm.outtakeDeposit()
                        ),
                        dropSample.build(),
                        outtakeServo.clawOpen(),
                        new ParallelAction(
                                slides.slidesTransfer(),
                                outtakeRotatingArm.outtakeTransfer()
                        ),
                        getMiddleSample.build()



                )
        );

    }
}