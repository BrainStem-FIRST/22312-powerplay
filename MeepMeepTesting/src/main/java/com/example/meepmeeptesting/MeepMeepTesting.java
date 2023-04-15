package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Bring in components from Auto class
        // Determine waypoints based on Alliance and Orientation
        //------------------------------------------------------
        //            Variable used for trajectories
        //------------------------------------------------------

        Pose2d startingPose, preloadPose, depositPose, pickupPose, parkingPose;

        // Determine waypoints based on Alliance and Orientation
        double  XFORM_X, XFORM_Y;
        double  preloadDeltaX, preloadDeltaY,
                pickupDeltaX, pickupDeltaY,
                depositDeltaX, depositDeltaY;
        double  startingHeading, startingTangent,
                preloadHeading, preloadTangent,
                depositHeading, depositTangent,
                pickupHeading, pickupTangent;

        // Orientation Adjustments
        XFORM_X = -1;
        XFORM_Y = -1;

        startingHeading = -90;
        startingTangent = 90;

        preloadHeading = 180;   //matching 1trajectory's values. Previously was 185;
        preloadTangent = 0;

        pickupHeading = 180;
        pickupTangent = 180;    //matching 1trajectory's values. Previously was 179;

        depositHeading = 180;
        depositTangent = 0;

        preloadDeltaX = 1.5;  // matching 1trajectory's values. Previously was 2;
        preloadDeltaY = 0;

        pickupDeltaX = -2;  // matching 1trajectory's values. Previously was 0;
        pickupDeltaY = 0;

        depositDeltaX = -0.5;
        depositDeltaY = 0;

        // Poses
        startingPose = new Pose2d(XFORM_X * 36, XFORM_Y * 63, Math.toRadians(startingHeading));
        preloadPose = new Pose2d(XFORM_X * (26 + preloadDeltaX), XFORM_Y * (11.5 + preloadDeltaY), Math.toRadians(preloadHeading)); //16, 11.5
        depositPose = new Pose2d(XFORM_X * (2.5 + depositDeltaX), XFORM_Y * (11.5 + depositDeltaY), Math.toRadians(depositHeading));
        pickupPose = new Pose2d(XFORM_X * (54-5 + pickupDeltaX), XFORM_Y * (11.5 + pickupDeltaY), Math.toRadians(pickupHeading));
        parkingPose     = new Pose2d(); // to be defined after reading the signal cone


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 75, Math.toRadians(180), Math.toRadians(180), 13.59)
                .setDimensions(14.25,14.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder((startingPose))

                                // Preload
                                .setTangent(Math.toRadians(startingTangent))
                                .splineToSplineHeading(preloadPose, Math.toRadians(preloadTangent))

                                .waitSeconds(2)

                                // Pickup 1
                                .setTangent(Math.toRadians(pickupTangent))
                                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
                                .forward(5)

                                .waitSeconds(1)

                                // Deposit 1
                                .setTangent(Math.toRadians(depositTangent))
                                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))

                                .waitSeconds(1)

                                // Pickup 2
                                .setTangent(Math.toRadians(pickupTangent))
                                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
                                .forward(5)

                                .waitSeconds(1)

                                // Deposit 2
                                .setTangent(Math.toRadians(depositTangent))
                                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))

                                .waitSeconds(1)

                                // Pickup 3
                                .setTangent(Math.toRadians(pickupTangent))
                                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
                                .forward(5)

                                .waitSeconds(1)

                                // Deposit 3
                                .setTangent(Math.toRadians(depositTangent))
                                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))

                                .waitSeconds(1)

                                // Pickup 4
                                .setTangent(Math.toRadians(pickupTangent))
                                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
                                .forward(5)

                                .waitSeconds(1)

                                // Deposit 4
                                .setTangent(Math.toRadians(depositTangent))
                                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))

                                .waitSeconds(1)

                                // Pickup 5
                                .setTangent(Math.toRadians(pickupTangent))
                                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
                                .forward(5)

                                .waitSeconds(1)

                                // Deposit 5
                                .setTangent(Math.toRadians(depositTangent))
                                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))

                                .waitSeconds(1)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_LIGHT) //.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}