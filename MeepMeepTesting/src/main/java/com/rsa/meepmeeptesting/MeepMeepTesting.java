package com.rsa.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity first = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(16, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(MeepMeepTesting::farMiddle);

        RoadRunnerBotEntity second = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(16, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(MeepMeepTesting::closeWall);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(first)
                .addEntity(second)
                .start();
    }

    public static TrajectorySequence farWall(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-42, -34, Math.toRadians(120)))
                .lineToSplineHeading(new Pose2d(-48, -40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(180))

                .setReversed(true)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(-24, -60), Math.toRadians(0))
                .lineTo(new Vector2d(12, -60))
                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(45))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(47, -31))
                .splineToConstantHeading(new Vector2d(0, -60), Math.toRadians(180))
                .lineTo(new Vector2d(-24, -60))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(180))

                .setReversed(true)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(-24, -60), Math.toRadians(0))
                .lineTo(new Vector2d(12, -60))
                .splineToConstantHeading(new Vector2d(48, -42), Math.toRadians(45))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(47, -43))
                .splineToConstantHeading(new Vector2d(0, -60), Math.toRadians(180))
                .lineTo(new Vector2d(-24, -60))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(180))

                .setReversed(true)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(-24, -60), Math.toRadians(0))
                .lineTo(new Vector2d(12, -60))
                .splineToConstantHeading(new Vector2d(48, -42), Math.toRadians(45))
                .waitSeconds(0.5)

                .strafeLeft(18)

                .build();
    }

    public static TrajectorySequence farMiddle(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-46, -16))
                .lineToSplineHeading(new Pose2d(-54, -12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-60, -12), Math.toRadians(180))

                .setReversed(true)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(270))
                .setReversed(false)
                .waitSeconds(0.5)
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                .setReversed(true)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(270))
                .setReversed(false)
                .waitSeconds(0.5)
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                .setReversed(true)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-36, -12), Math.toRadians(0))
                .lineTo(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(270))
                .setReversed(false)
                .waitSeconds(0.5)

                .strafeRight(18)

                .build();
    }

    public static TrajectorySequence closeWall(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(10, -36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(48, -30, Math.toRadians(180)))
                .waitSeconds(0.5)

                .lineTo(new Vector2d(47, -31))
                .splineToConstantHeading(new Vector2d(0, -60), Math.toRadians(180))
                .lineTo(new Vector2d(-24, -60))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(135))
                .setReversed(true)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-59, -37))
                .splineToConstantHeading(new Vector2d(-24, -60), Math.toRadians(0))
                .lineTo(new Vector2d(12, -60))
                .splineToConstantHeading(new Vector2d(48, -42), Math.toRadians(45))
                .waitSeconds(0.5)

                .lineTo(new Vector2d(47, -43))
                .splineToConstantHeading(new Vector2d(0, -60), Math.toRadians(180))
                .lineTo(new Vector2d(-24, -60))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(135))
                .setReversed(true)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-59, -37))
                .splineToConstantHeading(new Vector2d(-24, -60), Math.toRadians(0))
                .lineTo(new Vector2d(12, -60))
                .splineToConstantHeading(new Vector2d(48, -42), Math.toRadians(45))
                .waitSeconds(0.5)

                .strafeLeft(18)

                .build();
    }

    public static TrajectorySequence closeMiddle(DriveShim drive) {
        return drive.trajectorySequenceBuilder(new Pose2d(12, -64, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(10, -36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(48, -30, Math.toRadians(180)))

                .waitSeconds(0.5)
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))
                .setReversed(true)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(270))
                .setReversed(false)

                .waitSeconds(0.5)
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))
                .setReversed(true)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(270))
                .setReversed(false)

                .waitSeconds(0.5)
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineTo(new Vector2d(-36, -12))
                .splineTo(new Vector2d(-61, -20), Math.toRadians(200))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(-36, -12), Math.toRadians(0))
                .lineTo(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(270))
                .setReversed(false)

                .waitSeconds(0.5)

                .build();
    }






}

