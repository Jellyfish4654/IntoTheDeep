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
        ConceptAprilTagLocalization webcam = new ConceptAprilTagLocalization();

        TrajectoryActionBuilder moveForward = drive.actionBuilder(initialPose)
                .lineToY(0);




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("left target", Slides.leftTarget);
            telemetry.addData("left slide pos", slides.getCurrentLeftPosition());
            telemetry.addData("action running", actionRunning);
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        double[] coordinates = webcam.findCoordinates();
        Pose2d newPose = new Pose2d(coordinates[0], coordinates[1], coordinates[2]);

        TrajectoryActionBuilder moveToPlace = drive.actionBuilder(newPose)
                .strafeToLinearHeading(new Vector2d(41, 38.7), Math.toRadians(270));

        Actions.runBlocking(
                new SequentialAction(
                    moveToPlace.build()
                )
        );



    }
}