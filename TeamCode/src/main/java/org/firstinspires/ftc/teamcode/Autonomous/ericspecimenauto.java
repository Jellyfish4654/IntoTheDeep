package org.firstinspires.ftc.teamcode.Autonomous;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.Framework.Slides;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Config
@Autonomous(name = "ericsauto", group = "Autonomous")
public class ericspecimenauto extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        initHardware();
        Pose2d initialPose = new Pose2d(22, 61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // hang preloaded specimen
        // get sample
        // turn into specimen
        // hang specimen

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(42, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-15, 32.45), Math.toRadians(90));
        //to chamber to hang

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(0, 50), Math.toRadians(270));
        // to the wall for specimen... also should turn around so it can grab a new one

        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .lineToY(32.45 );
        //back to the specimen bar to hang


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
                                tab1.build(),
                                new SequentialAction(
                                        slides.slidesUnderBar(),
                                        outtakeRotatingArm.outtakeChamber()
                                )
                        ),
                        slides.slidesOverBar(),
                        outtakeServo.clawOpen(),
                        slides.slidesDown(),
//                        new ParallelAction(
//                                slides,
//                                tab2.build()
//                        ),
                        outtakeServo.clawClose(),
                        slides.slidesOverBar(),

                        tab4.build()






                )
        );

    }
}