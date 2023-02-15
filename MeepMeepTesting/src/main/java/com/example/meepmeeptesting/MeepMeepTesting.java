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

        preloadHeading = 180;
        preloadTangent = 0;

        pickupHeading = 180;
        pickupTangent = 180;

        depositHeading = 180;
        depositTangent = 0;

        preloadDeltaX = 0;
        preloadDeltaY = 0;

        pickupDeltaX = 0;
        pickupDeltaY = 0;

        depositDeltaX = 0;
        depositDeltaY = 0;

        // Poses
        startingPose    = new Pose2d(XFORM_X * 36, XFORM_Y * 63, Math.toRadians(startingHeading));
        preloadPose     = new Pose2d(XFORM_X * (18 + preloadDeltaX), XFORM_Y * (10.5 + preloadDeltaY), Math.toRadians(preloadHeading));
        depositPose     = new Pose2d(XFORM_X * (27 + depositDeltaX), XFORM_Y * (10.5 + depositDeltaY), Math.toRadians(depositHeading));
        pickupPose      = new Pose2d(XFORM_X * (52 + pickupDeltaX), XFORM_Y * (10.5 + pickupDeltaY), Math.toRadians(pickupHeading));
        parkingPose     = new Pose2d(); // to be defined after reading the signal cone


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 30, Math.toRadians(180), Math.toRadians(60), 3.5)
                .setDimensions(12.25,14.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder((startingPose))

                                // Cycle 1
                                .setTangent(Math.toRadians(startingTangent))

                                // 2.55 sec to reach destination
                                .splineToSplineHeading(preloadPose, Math.toRadians(preloadTangent),
                                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75),
                                        SampleMecanumDrive.getAccelerationConstraint(90))

                                .waitSeconds(2)

                                .setTangent(Math.toRadians(pickupTangent))

                                // Move to the cone stack head first, stop at arm's reach
                                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent),
                                        SampleMecanumDrive.getVelocityConstraint(25, Math.toRadians(180), 9.75),
                                        SampleMecanumDrive.getAccelerationConstraint(90))

                                .waitSeconds(2)

                                .setTangent(depositTangent)
                                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent),
                                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75),
                                        SampleMecanumDrive.getAccelerationConstraint(90))

                                .waitSeconds(2)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}