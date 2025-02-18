package org.firstinspires.ftc.teamcode.Autonomous;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.Framework.Slides;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Config
@Autonomous(name = "jocelynsauto", group = "Autonomous")
public class jocelynsampleauto extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        initHardware();
        Pose2d initialPose = new Pose2d(22, 61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(42, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-15, 35.3), Math.toRadians(90));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(32.45 );
        //to the chamber
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToY(32.45 );
        //back to the wall

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
                        tab1.build(),
                        slides.slidesUnderBar(),
                        outtakeRotatingArm.outtakeChamber(),
                        tab2.build(),
                        slides.slidesOverBar(),
                        outtakeServo.clawOpen(),
                         // back to the wall to grab another specimen
                        new ParallelAction(
                                tab3.build(),
                                outtakeRotatingArm.outtakeGrabSpecimen()

                        )
                )
        );
    }
}