//property of erk
package org.firstinspires.ftc.teamcode.Autonomous.Old;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class JellyBotTest extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-23.5, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        int visionOutputPosition = 1;
        Actions.runBlocking(slides.slidesDown());
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(42)
                .setTangent(Math.toRadians(0))
                .lineToX(-5)
                .setTangent(Math.toRadians(90))
                .lineToY(35.3)
                .waitSeconds(3) // hang specimen
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(-48.2)
                .setTangent(Math.toRadians(270))
                .waitSeconds(2) // grab sample
                .lineToY(55.3)
                .waitSeconds(2) // drop sample
                .lineToY(45.3)
                .setTangent(Math.toRadians(180))
                .lineToX(-58.2)
                .setTangent(Math.toRadians(270))
                .waitSeconds(2) // grab sample
                .lineToY(61.3)
                .waitSeconds(2); // park
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }
        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }
        Actions.runBlocking(
                new SequentialAction(
                        slides.slidesUp(),
                        intakeServo.clawOpen(),
                        outtakeServo.clawClose(),
                        extendo.extendoExtend(),
                        wrist.wristUp(),
                        trajectoryActionCloseOut
                )
        );
    }
}
