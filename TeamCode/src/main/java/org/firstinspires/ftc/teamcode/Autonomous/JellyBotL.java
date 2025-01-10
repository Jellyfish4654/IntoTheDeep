package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "JellyBotL", group = "Autonomous")
public class JellyBotL extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(23.5, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Actions.runBlocking(slides.slidesDown());
        Actions.runBlocking(outtakeServo.clawClose());

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(initialPose)
                .lineToY(42)
                .setTangent(Math.toRadians(180))
                .lineToX(5)
                .setTangent(Math.toRadians(90))
                .lineToY(35.3);

        Pose2d secondPose = new Pose2d(5, 35.3, Math.toRadians(90));
        TrajectoryActionBuilder toSample = drive.actionBuilder(secondPose)
                .waitSeconds(3) // hang specimen
                .lineToY(45.3)
                .setTangent(Math.toRadians(0))
                .lineToX(48.2)
                .setTangent(Math.toRadians(270));

        Pose2d thirdPose = new Pose2d(48.2, 45.3, Math.toRadians(270));
        TrajectoryActionBuilder waitForExtendo = drive.actionBuilder(thirdPose)
                .waitSeconds(1);
        TrajectoryActionBuilder toBasket = drive.actionBuilder(thirdPose)
                .waitSeconds(2) // grab sample
                .lineToY(53.2)
                .setTangent(Math.toRadians(180))
                .lineToX(51.8)
                .setTangent(Math.toRadians(225));

        Pose2d fourthPose = new Pose2d(51.8, 53.2, Math.toRadians(225));
        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(fourthPose)
                .waitSeconds(3) // basket
                .setTangent(Math.toRadians(180))
                .lineToX(48.2)
                .setTangent(Math.toRadians(270))
                .lineToY(45.3)
                .setTangent(Math.toRadians(0))
                .lineToX(58.2)
                .setTangent(Math.toRadians(270));

        Pose2d fifthPose = new Pose2d(58.2, 45.3, Math.toRadians(270));
        TrajectoryActionBuilder toSecondBasket = drive.actionBuilder(fifthPose)
                .waitSeconds(2) // grab sample
                .setTangent(Math.toRadians(180))
                .lineToX(51.8)
                .setTangent(Math.toRadians(90))
                .lineToY(53.2)
                .setTangent(Math.toRadians(225));


        waitForStart();
        if (isStopRequested()) return;
        Action action1 = toSubmersible.build();
        Action action2 = toSample.build();
        Action action3 = toBasket.build();
        Action action4 = toSecondSample.build();
        Action action5 = toSecondBasket.build();

        Actions.runBlocking(
                new SequentialAction(
                        action1,
                        outtakeRotatingArmServos.outtakeDeposit(),
                        outtakeServo.clawClose(),
                        action2,
                        extendo.extendoExtend(),
                        intakeServo.clawClose(),
                        extendo.extendoRetract(),
                        wrist.wristUp(),
                        outtakeRotatingArmServos.outtakeTransfer(),
                        outtakeServo.clawClose(),
                        intakeServo.clawOpen(),
                        action3,
                        slides.slidesUp(),
                        outtakeRotatingArmServos.outtakeDeposit(),
                        outtakeServo.clawOpen(),
                        outtakeRotatingArmServos.outtakeTransfer(),
                        action4,
                        wrist.wristDown(),
                        extendo.extendoExtend(),
                        intakeServo.clawClose(),
                        wrist.wristUp(),
                        extendo.extendoRetract(),
                        outtakeServo.clawClose(),
                        intakeServo.clawOpen(),
                        action5,
                        outtakeRotatingArmServos.outtakeDeposit(),
                        outtakeServo.clawOpen()
                )
        );
    }
}
