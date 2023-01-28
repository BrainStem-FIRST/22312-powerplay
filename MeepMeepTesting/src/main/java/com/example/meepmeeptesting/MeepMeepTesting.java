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

        Pose2d startingPose, depositPose, pickupPose, parkingPose;
        Pose2d cornerPose;

        // Determine waypoints based on Alliance and Orientation
        double  XFORM_X, XFORM_Y;
        double  pickupDeltaX, pickupDeltaY,
                cornerDeltaX, cornerDeltaY,
                depositDeltaX, depositDeltaY;
        double  startingHeading, startingTangent,
                depositHeading, depositTangent,
                pickupHeading, pickupTangent,
                cornerHeading, cornerTangent;

        // Orientation Adjustments
        XFORM_X = -1;
        XFORM_Y = -1;

        startingHeading = -90;
        startingTangent = 90;

        cornerHeading = 180;
        cornerTangent = 90;

        depositHeading = 180;
        depositTangent = 0;

        pickupHeading = 180;
        pickupTangent = 180;

        cornerDeltaX = 0;
        cornerDeltaY = 0;

        pickupDeltaX = 0; // previously 0
        pickupDeltaY = 0;  // previously 0

        depositDeltaX = 0;
        depositDeltaY = 0;

        // Poses
        startingPose    = new Pose2d(XFORM_X * 36, XFORM_Y * 63.75, Math.toRadians(startingHeading));
        cornerPose      = new Pose2d(XFORM_X * (30 + cornerDeltaX), XFORM_Y * (40 + cornerDeltaY), Math.toRadians(cornerHeading));
        depositPose     = new Pose2d(XFORM_X * (21 + cornerDeltaX), XFORM_Y * (12 + cornerDeltaY), Math.toRadians(depositHeading)); //depositX-28
        pickupPose      = new Pose2d(XFORM_X * (54 + pickupDeltaX), XFORM_Y * (12 + pickupDeltaY), Math.toRadians(pickupHeading));
        parkingPose     = new Pose2d(); // to be defined after reading the signal cone


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 30, Math.toRadians(180), Math.toRadians(60), 3.5)
                .setDimensions(12.25,14.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder((startingPose))

                                ///////////// Best Time: 3.34 sec ///////////////
                                .setTangent(Math.toRadians(startingTangent))
//                                .splineToLinearHeading(cornerPose,Math.toRadians(90),
//                                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75), //3.5),
//                                        SampleMecanumDrive.getAccelerationConstraint(90))

                                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent),
                                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(270), 9.75), //3.5),
                                        SampleMecanumDrive.getAccelerationConstraint(90))

                                // Cycle 1
                                .lineToLinearHeading(pickupPose,
                                        SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(270), 9.75),
                                        SampleMecanumDrive.getAccelerationConstraint(90))

                                .lineToConstantHeading(new Vector2d(depositPose.getX(), depositPose.getY()),
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