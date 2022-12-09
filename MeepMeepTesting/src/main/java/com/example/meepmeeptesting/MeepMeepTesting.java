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
        double  XFORM_X=-1, XFORM_Y=-1;
        double  pickupDeltaX=0, pickupDeltaY=0,
                depositDeltaX=0, depositDeltaY=0,
                preloadDeltaX=0, preloadDeltaY=0,
                cornerDeltaX=0, cornerDeltaY=0;
        double  startingHeading=90, deliveryHeading=180, pickupHeading=180;

        Pose2d startingPose, pickupPose, depositPose, parkingPose;
        Pose2d cornerPose;

//        startingPose    = new Pose2d(XFORM_X * 35.5, XFORM_Y * 63.75, Math.toRadians(startingHeading));
//        cornerPose      = new Pose2d(XFORM_X * (56 + cornerDeltaX), XFORM_Y * (59 + pickupDeltaY), Math.toRadians(0));
//        pickupPose      = new Pose2d(XFORM_X * (55.75 + pickupDeltaX), XFORM_Y * (12 + pickupDeltaY), Math.toRadians(240));

        startingPose    = new Pose2d(XFORM_X * 35.5, XFORM_Y * 63.75, Math.toRadians(startingHeading));
        cornerPose      = new Pose2d(XFORM_X * (60 + cornerDeltaX), XFORM_Y * (55 + pickupDeltaY), Math.toRadians(-90));
        pickupPose      = new Pose2d(XFORM_X * (56.75 + pickupDeltaX), XFORM_Y * (12 + pickupDeltaY), Math.toRadians(240));
//        depositPose     = new Pose2d(XFORM_X * (24 + depositDeltaX), XFORM_Y * (10 + depositDeltaY), Math.toRadians(deliveryHeading));
        parkingPose     = new Pose2d(); // to be defined after reading the signal cone


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(28, 30, Math.toRadians(180), Math.toRadians(60), 3.5)
                .setDimensions(12.25,14.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder((startingPose))

                                .setTangent(Math.toRadians(120))
                                .splineToLinearHeading(cornerPose,Math.toRadians(90),
                                        SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(180), 3.5),
                                        SampleMecanumDrive.getAccelerationConstraint(30))
                                .setTangent(Math.toRadians(90))
                                .lineToLinearHeading(pickupPose) //,Math.toRadians(60))






                                //moves forward in a line facing 90 degrees away (positioned in between two poles)
//                                .lineToLinearHeading(new Pose2d(cornerPose.getX()+(XFORM_X*cornerDeltaX), cornerPose.getY()+(XFORM_Y*(cornerDeltaY)), 180))
//                                        .lineToLinearHeading(new Pose2d(startingPose.getX()+(XFORM_X*preloadDeltaX), XFORM_Y * (19+preloadDeltaY), pickupPose.getHeading()),
//                                        SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
//                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))

//                                .addTemporalMarker(0.3, () -> {    // Start positioning scaffolding
//                                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
//                                    robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_LOWPOLE);
//                                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
//                                    robot.turret.moveTo(-200); //TODO: Add states for auto pickup and delivery
//                                })

//                                .lineToLinearHeading(new Pose2d(pickupPose.getX() + XFORM_X * 2, pickupPose.getY() + XFORM_Y * 2, pickupPose.getHeading()))
//                                        SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
//                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL)) //TODO: FIX PICKUP POSE

                                // stop swinging
//                                .waitSeconds(.1)

//                                 Drop off the cone at hand on the Low pole on the left
//                                .addTemporalMarker(() -> {
//                                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.PLACEMENT_HEIGHT);
//                                    robot.grabber.grabberOpen();
//                                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.APPROACH_HEIGHT);
//                                })
//
//
//                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
//                                    robot.lift.raiseHeightTo(robot.lift.liftPositionPickup);
//                                    robot.turret.moveTo(200); //TODO: Add states for auto pickup and delivery
//                                })

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}