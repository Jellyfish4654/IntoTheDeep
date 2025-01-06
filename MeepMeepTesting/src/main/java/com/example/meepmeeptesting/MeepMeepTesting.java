package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-23.5, 62, Math.toRadians(270)))
                        .waitSeconds(3)
                        .build());
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(23.5, 62, Math.toRadians(270)))
                        .forward(20)
                        .turn(Math.toRadians(-90))
                        .forward(18.5)
                        .turn(Math.toRadians(-90))
                        .forward(-6.7)
                        .waitSeconds(3) //hang specimen
                        .forward(10)
                        .turn(Math.toRadians(-90))
                        .forward(43.2)
                        .turn(Math.toRadians(-90))
                        .waitSeconds(2) //grab sample
                        .forward(-4.3)
                        .turn(Math.toRadians(-45))
                        .forward(-5)
                        .waitSeconds(3) // basket
                        .forward(5)
                        .turn(Math.toRadians(45))
                        .forward(4.3)
                        .turn(Math.toRadians(90))
                        .forward(10)
                        .turn(Math.toRadians(-90))
                        .waitSeconds(2) //grab sample
                        .turn(Math.toRadians(-90))
                        .forward(10)
                        .turn(Math.toRadians(-90))
                        .forward(4.3)
                        .turn(Math.toRadians(135))
                        .forward(-5)
                        .waitSeconds(3) // basket
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .start();
    }
}