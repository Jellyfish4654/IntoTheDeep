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
import org.firstinspires.ftc.teamcode.Framework.Algorithms.ConceptAprilTagLocalization;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.Framework.Hardware.Slides;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Config
@Autonomous(name = "jeffreyauto", group = "Autonomous")
public class jeffreyauto extends BaseOpMode {


    public boolean actionRunning = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.


        initHardware();
        Pose2d initialPose = new Pose2d(0, 60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder moveForward = drive.actionBuilder(initialPose)
                .lineToY(0);

        Pose2d initinitialPose = new Pose2d(0, 0, Math.toRadians(90));
        TrajectoryActionBuilder moveToPlace = drive.actionBuilder(initinitialPose)
                .lineToY(0);

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
                    moveForward.build(),
                        new ParallelAction(
                        )
                )
        );

    }
}