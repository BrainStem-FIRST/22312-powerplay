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

        Pose2d startingPose, pickupPose, parkingPose;
        Pose2d cornerPose;

        // Determine waypoints based on Alliance and Orientation
        double  XFORM_X, XFORM_Y;
        double  pickupDeltaX, pickupDeltaY,
                cornerDeltaX, cornerDeltaY;
        double  startingHeading, startingTangent,
                pickupHeading, pickupTangent,
                cornerHeading, cornerTangent;

        // Orientation Adjustments
        XFORM_X = -1;
        XFORM_Y = -1;

        startingHeading = -90;
        startingTangent = 135;

        cornerHeading = -90;
        cornerTangent = 90;

        pickupHeading = -90;
        pickupTangent = 90;

        cornerDeltaX = 0;
        cornerDeltaY = 0;

        pickupDeltaX = 0; // previously 0
        pickupDeltaY = 0;  // previously 0


        // Poses
        startingPose    = new Pose2d(XFORM_X * 36, XFORM_Y * 63.75, Math.toRadians(startingHeading));
        cornerPose      = new Pose2d(XFORM_X * (60 + cornerDeltaX), XFORM_Y * (52 + cornerDeltaY), Math.toRadians(cornerHeading));
        pickupPose      = new Pose2d(XFORM_X * (56.26 + pickupDeltaX), XFORM_Y * (13.75 + pickupDeltaY), Math.toRadians(pickupHeading));
        parkingPose     = new Pose2d(); // to be defined after reading the signal cone


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 30, Math.toRadians(180), Math.toRadians(60), 3.5)
                .setDimensions(12.25,14.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder((startingPose))

                                ///////////// Best Time: 3.34 sec ///////////////
                                .setTangent(Math.toRadians(startingTangent))
                                .splineToConstantHeading(new Vector2d(cornerPose.getX(), cornerPose.getY()), Math.toRadians(cornerTangent),
                                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75), //3.5),
                                        SampleMecanumDrive.getAccelerationConstraint(90))

//                                .setTangent(Math.toRadians(pickupTangent))
                                .splineToConstantHeading(new Vector2d(pickupPose.getX(), pickupPose.getY()),Math.toRadians(pickupTangent),
                                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75), //3.5),
                                        SampleMecanumDrive.getAccelerationConstraint(90))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}