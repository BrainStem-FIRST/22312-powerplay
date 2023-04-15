package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Vision.imagecv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.autoclasses.BrainStemRobotA;
import org.firstinspires.ftc.teamcode.robot.autoclasses.ConstantsA;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Config
@Autonomous(name="Robot: Auto Uncontested BETA", group="Robot")

public class auto_uncontested_BETA extends LinearOpMode {
    //camera
    public OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    public int Ending_Location = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 1000;
    double cx = 100;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.00037;

    // Corresponds to parking zones
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null; //camera

    // Trajectory related variables
    public static boolean isAllianceRED;
    private boolean allianceIsSet = false;
    public static boolean isOrientationLEFT;
    private boolean orientationIsSet = false;
    private boolean programConfirmation = false;

    // original auto
    public static int PARKING_NUMBER = 3; // Controlled by the dashboard for test purposes
    public static double SPEED = 60.0;    // Controlled by the dashboard for test purposes
    private ElapsedTime autoTime = new ElapsedTime();
    private double TIME_TO_PARK = 29;  //27

    // used for trajectory state machine
    enum    TrajectoryState {
        TRAJECTORY_RUNNING_STATE,
        TRAJECTORY_PARKING_STATE,
        TRAJECTORY_IDLE
    }
    TrajectoryState currentTrajectoryState = TrajectoryState.TRAJECTORY_RUNNING_STATE;

    Map<String, String> stateMap = new HashMap<String, String>() { };
    ConstantsA constants = new ConstantsA();


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


    //------------------------------------------------------
    //            Define trajectories
    //------------------------------------------------------

    // From starting position, turn sideways (to push the signal cone), strafe, deliver on Low Pole, spline to pickup position, and pickup top-most cone

    TrajectorySequence buildTrajectory(BrainStemRobotA robot) {
        TrajectorySequence trajectory;

        trajectory = robot.drive.trajectorySequenceBuilder(startingPose)

                //make sure the cone is lifted a little from the ground before robot starts moving
                .addTemporalMarker(0.2, () -> {
                    // lift off the ground for transportation
                    robot.lift.goToClear();
                })

                .addTemporalMarker(0.3, () -> {
                    robot.alignment.alignDown();
                })

                .setTangent(Math.toRadians(startingTangent))

                // 2.55 sec to reach destination
                .splineToSplineHeading(preloadPose, Math.toRadians(preloadTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 13.59),
//                        SampleMecanumDrive.getAccelerationConstraint(95))


                // Timer is from start of the trajectory; it is not an offset
                .addTemporalMarker(1.0, () -> {
                    robot.arm.extendHome();
                    robot.lift.goToHighPoleHeight();
                })

                .addTemporalMarker(1.7, () -> {
                    robot.turret.gotoPreloadPosition();
                })

                // All the subsystem movements will have finished before the robot stops at the preload position
                // Once the robot stops, extend the arm.
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_PRELOAD);
                })

                // Add a timer here to catchup with the subsystem movement after the robot stopped
                // This is the time that passes between the robot positioned itself next to
                // the high pole and the arm was extended positioning the cone right above the pole

                // Time for the arm to reach top of the pole
                .waitSeconds(0.2) // TODO: adjust time value so right after it the drop cone can start

                // Drop Cone
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.lift.raiseHeightTo(robot.lift.getPosition() - 150);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    robot.grabber.grabberOpen();
                })

                // Pull extension immediately, and turn turret afterwards when the robot started moving
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {
                    robot.arm.extendHome();
                })

                // Add time to cover the duration between the end of previous marker (.waitseconds) and
                // the start of the robot moving for the next pickup cycle.
                //
                // This should account for duration of drop cone cycle and the time necessary to wait for extendHome
                .waitSeconds(0.4)   // TODO: Fine tune this duration to adjust start of the robot's move


                /********* PICKUP CYCLE 1 **********/

                // Move turret at the same time as robot start moving away from high pole,
                // which is the end of previous .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    robot.turret.goToPickupPosition();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () ->{
                    robot.alignment.alignUp();
                })
                // lower the lift after the turret repositioned and before the robot reached its target
                // offset is relative to when robot reached its destination
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.lift.goToPickupHeight(0);
                })

                // note that his movement starts at offset 0 following the last .waitseconds (i.e. not after alignUp)
                .setTangent(Math.toRadians(pickupTangent))
                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(75))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(40,Math.toRadians(180),13.59),SampleMecanumDrive.getAccelerationConstraint(75))


                // Reach arm to touch the cone after the robot stopped
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_PICKUP);
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> {
                    robot.grabber.grabberOpenWide();
                })

                //Pickup Cone
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.grabber.grabberClose();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> {
                    robot.lift.goToClear();
                })

                // This is the duration the robot waits at the pickup station (while its subsystems are picking the cone up)
                .waitSeconds(0.2)

                /********** DEPOSIT CYCLE 1 ***********/

                // The drivetrain move is placed in the trajectory after the subsystem sequence so that
                // the subsystem marker offsets can be calculated from the end of the .waitseconds that
                // marks the end of the pickup cycle.

                // Timer is offset from the end of .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.extendHome();
                })

                // Timer is from start of the trajectory; it is not an offset
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.lift.goToHighPoleHeight();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () ->{
                    robot.alignment.alignDown();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.turret.goToDepositPosition();
                })

                // This drivetrain move will start after the .waitsecond that marks the end of the pickup cycle.
                // It is independent of the subsystem moves and will be concurrent with the above marker offsets.
                // Note that the robot will stop before the subsystem moves are complete. Need to add an idle time
                // after the drivetrain command to mark the start of the deposit cone cycle.

                .setTangent(Math.toRadians(depositTangent))
                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(95))

                // Add a timer here to catchup with the subsystem movement after the robot stopped
                // This is the time that passes between the robot positioned itself next to
                // the high pole and lift/turret finished moving right before extending the arm
                .waitSeconds(0.0) // TODO: adjust time value so right after it arm can be extended

                // Extend arm only after the lift completed its rise
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
                })

                // Drop Cone
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    robot.lift.raiseHeightTo(robot.lift.getPosition() - 150);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()-> {
                    robot.grabber.grabberOpen();
                })

                // Pull extension immediately, and turn turret afterwards when the robot started moving
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.arm.extendHome();
                })

                // Add time to cover the duration between the end of previous marker (.waitseconds) and
                // the start of the robot moving for the next pickup cycle.

                // This should account for duration of drop cone cycle and the time necessary to wait for extendHome
                .waitSeconds(0.6)   // TODO: Fine tune this duration to adjust start of the robot's move

                /********* PICKUP CYCLE 2 *********/

                // Move turret at the same time as robot start moving away from high pole,
                // which is the end of previous .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    robot.turret.goToPickupPosition();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () ->{
                    robot.alignment.alignUp();
                })

                // lower the lift after the turret repositioned and before the robot reached its target
                // offset is relative to when robot reached its destination
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.lift.goToPickupHeight(1);
                })

                // note that his movement starts at offset 0 following the last .waitseconds (i.e. not after alignUp)
                .setTangent(Math.toRadians(pickupTangent))
                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(75))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(40,Math.toRadians(180),13.59),SampleMecanumDrive.getAccelerationConstraint(75))


                // Reach arm to touch the cone after the robot stopped
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_PICKUP);
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> {
                    robot.grabber.grabberOpenWide();
                })

                // Pickup Cone
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.grabber.grabberClose();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> {
                    robot.lift.goToClear();
                })

                // This is the duration the robot waits at the pickup station (while its subsystems are picking the cone up)
                .waitSeconds(0.2)


                /********* DEPOSIT CYCLE 2 *********/

                // The drivetrain move is placed in the trajectory after the subsystem sequence so that
                // the subsystem marker offsets can be calculated from the end of the .waitseconds that
                // marks the end of the pickup cycle.

                // Timer is offset from the end of .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.extendHome();
                })

                // Timer is from start of the trajectory; it is not an offset
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.lift.goToHighPoleHeight();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () ->{
                    robot.alignment.alignDown();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.turret.goToDepositPosition();
                })

                // This drivetrain move will start after the .waitsecond that marks the end of the pickup cycle.
                // It is independent of the subsystem moves and will be concurrent with the above marker offsets.
                // Note that the robot will stop before the subsystem moves are complete. Need to add an idle time
                // after the drivetrain command to mark the start of the deposit cone cycle.

                .setTangent(Math.toRadians(depositTangent))
                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(95))

                // Add a timer here to catchup with the subsystem movement after the robot stopped
                // This is the time that passes between the robot positioned itself next to
                // the high pole and lift/turret finished moving right before extending the arm
                .waitSeconds(0.0) // TODO: adjust time value so right after it arm can be extended

                // Extend arm only after the lift completed its rise
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
                })

                // Drop Cone
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    robot.lift.raiseHeightTo(robot.lift.getPosition() - 150);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()-> {
                    robot.grabber.grabberOpen();
                })

                // Pull extension immediately, and turn turret afterwards when the robot started moving
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.arm.extendHome();
                })

                // Add time to cover the duration between the end of previous marker (.waitseconds) and
                // the start of the robot moving for the next pickup cycle.

                // This should account for duration of drop cone cycle and the time necessary to wait for extendHome
                .waitSeconds(0.6)   // TODO: Fine tune this duration to adjust start of the robot's move


                /********* PICKUP CYCLE 3 *********/


                // Move turret at the same time as robot start moving away from high pole,
                // which is the end of previous .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    robot.turret.goToPickupPosition();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () ->{
                    robot.alignment.alignUp();
                })

                // lower the lift after the turret repositioned and before the robot reached its target
                // offset is relative to when robot reached its destination
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.lift.goToPickupHeight(2);
                })

                // note that his movement starts at offset 0 following the last .waitseconds (i.e. not after alignUp)
                .setTangent(Math.toRadians(pickupTangent))
                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(75))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(40,Math.toRadians(180),13.59),SampleMecanumDrive.getAccelerationConstraint(75))


                // Reach arm to touch the cone after the robot stopped
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_PICKUP);
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> {
                    robot.grabber.grabberOpenWide();
                })

                // Pickup Cone
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.grabber.grabberClose();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> {
                    robot.lift.goToClear();
                })

                // This is the duration the robot waits at the pickup station (while its subsystems are picking the cone up)
                .waitSeconds(0.2)


                /********* DEPOSIT CYCLE 3 *********/

                // The drivetrain move is placed in the trajectory after the subsystem sequence so that
                // the subsystem marker offsets can be calculated from the end of the .waitseconds that
                // marks the end of the pickup cycle.

                // Timer is offset from the end of .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.extendHome();
                })

                // Timer is from start of the trajectory; it is not an offset
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.lift.goToHighPoleHeight();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () ->{
                    robot.alignment.alignDown();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.turret.goToDepositPosition();
                })

                // This drivetrain move will start after the .waitsecond that marks the end of the pickup cycle.
                // It is independent of the subsystem moves and will be concurrent with the above marker offsets.
                // Note that the robot will stop before the subsystem moves are complete. Need to add an idle time
                // after the drivetrain command to mark the start of the deposit cone cycle.

                .setTangent(Math.toRadians(depositTangent))
                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(95))

                // Add a timer here to catchup with the subsystem movement after the robot stopped
                // This is the time that passes between the robot positioned itself next to
                // the high pole and lift/turret finished moving right before extending the arm
                .waitSeconds(0.0) // TODO: adjust time value so right after it arm can be extended

                // Extend arm only after the lift completed its rise
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
                })

                // Drop Cone
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {
                    robot.lift.raiseHeightTo(robot.lift.getPosition() - 150);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.grabber.grabberOpen();
                })

                // Pull extension immediately, and turn turret afterwards when the robot started moving
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> {
                    robot.arm.extendHome();
                })

                // Add time to cover the duration between the end of previous marker (.waitseconds) and
                // the start of the robot moving for the next pickup cycle.

                // This should account for duration of drop cone cycle and the time necessary to wait for extendHome
                .waitSeconds(0.6)   // TODO: Fine tune this duration to adjust start of the robot's move


                /********* PICKUP CYCLE 4 *********/

                // Move turret at the same time as robot start moving away from high pole,
                // which is the end of previous .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    robot.turret.goToPickupPosition();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () ->{
                    robot.alignment.alignUp();
                })

                // lower the lift after the turret repositioned and before the robot reached its target
                // offset is relative to when robot reached its destination
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.lift.goToPickupHeight(3);
                })

                // note that his movement starts at offset 0 following the last .waitseconds (i.e. not after alignUp)
                .setTangent(Math.toRadians(pickupTangent))
                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(75))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(40,Math.toRadians(180),13.59),SampleMecanumDrive.getAccelerationConstraint(75))


                // Reach arm to touch the cone after the robot stopped
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_PICKUP);
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> {
                    robot.grabber.grabberOpenWide();
                })

                // Pickup Cone
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.grabber.grabberClose();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> {
                    robot.lift.goToClear();
                })

                // This is the duration the robot waits at the pickup station (while its subsystems are picking the cone up)
                .waitSeconds(0.2)


                /********* DEPOSIT CYCLE 4 *********/

                // The drivetrain move is placed in the trajectory after the subsystem sequence so that
                // the subsystem marker offsets can be calculated from the end of the .waitseconds that
                // marks the end of the pickup cycle.

                // Timer is offset from the end of .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.extendHome();
                })

                // Timer is from start of the trajectory; it is not an offset
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.lift.goToHighPoleHeight();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () ->{
                    robot.alignment.alignDown();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.turret.goToDepositPosition();
                })

                // This drivetrain move will start after the .waitsecond that marks the end of the pickup cycle.
                // It is independent of the subsystem moves and will be concurrent with the above marker offsets.
                // Note that the robot will stop before the subsystem moves are complete. Need to add an idle time
                // after the drivetrain command to mark the start of the deposit cone cycle.

                .setTangent(Math.toRadians(depositTangent))
                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(95))

                // Add a timer here to catchup with the subsystem movement after the robot stopped
                // This is the time that passes between the robot positioned itself next to
                // the high pole and lift/turret finished moving right before extending the arm
                .waitSeconds(0.0) // TODO: adjust time value so right after it arm can be extended

                // Extend arm only after the lift completed its rise
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
                })

                // Drop Cone
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {
                    robot.lift.raiseHeightTo(robot.lift.getPosition() - 150);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.grabber.grabberOpen();
                })

                // Pull extension immediately, and turn turret afterwards when the robot started moving
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> {
                    robot.arm.extendHome();
                })

                // Add time to cover the duration between the end of previous marker (.waitseconds) and
                // the start of the robot moving for the next pickup cycle.

                // This should account for duration of drop cone cycle and the time necessary to wait for extendHome
                .waitSeconds(0.6)   // TODO: Fine tune this duration to adjust start of the robot's move


                /********* PICKUP CYCLE 5 *********/

                // Move turret at the same time as robot start moving away from high pole,
                // which is the end of previous .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    robot.turret.goToPickupPosition();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () ->{
                    robot.alignment.alignUp();
                })

                // lower the lift after the turret repositioned and before the robot reached its target
                // offset is relative to when robot reached its destination
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.lift.goToPickupHeight(4);
                })

                // note that his movement starts at offset 0 following the last .waitseconds (i.e. not after alignUp)
                .setTangent(Math.toRadians(pickupTangent))
                .splineToSplineHeading(pickupPose, Math.toRadians(pickupTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(75))
                .forward(5, SampleMecanumDrive.getVelocityConstraint(40,Math.toRadians(180),13.59),SampleMecanumDrive.getAccelerationConstraint(75))

                // Reach arm to touch the cone after the robot stopped
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()-> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_PICKUP);
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.1,()-> {
                    robot.grabber.grabberOpenWide();
                })

                // Pickup Cone
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.grabber.grabberClose();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.05, ()-> {
                    robot.lift.goToClear();
                })

                // This is the duration the robot waits at the pickup station (while its subsystems are picking the cone up)
                .waitSeconds(0.2)


                /********* DEPOSIT CYCLE 5 *********/

                // The drivetrain move is placed in the trajectory after the subsystem sequence so that
                // the subsystem marker offsets can be calculated from the end of the .waitseconds that
                // marks the end of the pickup cycle.

                // Timer is offset from the end of .waitseconds
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.extendHome();
                })

                // Timer is from start of the trajectory; it is not an offset
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.lift.goToHighPoleHeight();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () ->{
                    robot.alignment.alignDown();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.turret.gotoPreloadPosition();
                })

                // This drivetrain move will start after the .waitsecond that marks the end of the pickup cycle.
                // It is independent of the subsystem moves and will be concurrent with the above marker offsets.
                // Note that the robot will stop before the subsystem moves are complete. Need to add an idle time
                // after the drivetrain command to mark the start of the deposit cone cycle.

                .setTangent(Math.toRadians(preloadTangent))
                .splineToSplineHeading(preloadPose, Math.toRadians(preloadTangent))
//                        SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), 9.75),
//                        SampleMecanumDrive.getAccelerationConstraint(95))

                // Add a timer here to catchup with the subsystem movement after the robot stopped
                // This is the time that passes between the robot positioned itself next to
                // the high pole and lift/turret finished moving right before extending the arm
                .waitSeconds(0.0) // TODO: adjust time value so right after it arm can be extended

                // Extend arm only after the lift completed its rise
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
                })

                // Drop Cone
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {
                    robot.lift.raiseHeightTo(robot.lift.getPosition() - 150);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    robot.grabber.grabberOpen();
                })

                // Pull extension immediately, and turn turret afterwards when the robot started moving
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()-> {
                    robot.arm.extendHome();
                })

                // Add time to cover the duration between the end of previous marker (.waitseconds) and
                // the start of the robot moving for the next pickup cycle.

                // This should account for duration of drop cone cycle and the time necessary to wait for extendHome
                .waitSeconds(0.6)   // TODO: Fine tune this duration to adjust start of the robot's move


                .build();

        return trajectory;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //------------------------------------------------------
        //                 Initialize the robot
        //------------------------------------------------------

        BrainStemRobotA robot = new BrainStemRobotA(hardwareMap, telemetry, stateMap);

        // Open grabber to allow Driver to load the initial cone
        robot.grabber.grabberOpen();

        // Wait until Alliance and Orientation is set by drivers
        while (!programConfirmation && !isStopRequested()) {
            setProgram();
        }

        //------------------------------------------------------
        //            Camera initialization
        //------------------------------------------------------

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam-1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50); //camera


        // Determine trajectory headings for all alliance combinations
        if (isOrientationLEFT) { // LEFT

            ///////////////////////////////////////////
            //             DO NOT CHANGE             //
            ///////////////////////////////////////////

            XFORM_X = -1;
            XFORM_Y = -1;

            startingHeading = -90;
            startingTangent = 90;

            preloadHeading = 180;
            preloadTangent = 0;

            pickupHeading = 180;
            pickupTangent = 180;

            depositHeading = 225;   // TODO: For BETA: Robot is angled 45deg so that the turret can reach uncontested high pole
            depositTangent = 0;

            ///////////////////////////////////////////
            //  CHANGE ONLY IF ABSOLUTELY NECESSARY  //
            //           DURING TOURNAMENT           //
            ///////////////////////////////////////////

            robot.turret.turret_PRELOAD_POSITION_VALUE = 170;
            robot.turret.turret_PICKUP_POSITION_VALUE = 0;
            robot.turret.turret_DEPOSIT_POSITION_VALUE = -290;  //270 //hitting hard stop

            robot.arm.EXTENSION_POSITION_PICKUP = 0;
            robot.arm.EXTENSION_POSITION_PRELOAD = 0.40;  // it was 0.49; extending a little more to hit the pole
            robot.arm.EXTENSION_POSITION_DEPOSIT = 0.40; //0.64

            ///////////////////////////////////////////
            //      MAKE ADJUSTMENTS ON POSES        //
            //           DURING TOURNAMENT           //
            ///////////////////////////////////////////

            preloadDeltaX = 0;
            preloadDeltaY = 0;

            pickupDeltaX = -1;
            pickupDeltaY = 0;

            depositDeltaX = 0;
            depositDeltaY = 0;
        } else {                  // RIGHT

            ///////////////////////////////////////////
            //             DO NOT CHANGE             //
            ///////////////////////////////////////////

            XFORM_X = 1;
            XFORM_Y = -1;

            startingHeading = -90;
            startingTangent = 90;

            preloadHeading = 0;
            preloadTangent = 180;

            pickupHeading = 0;
            pickupTangent = 0;

            depositHeading = -45;   // TODO: For BETA: Robot is angled 45deg so that the turret can reach uncontested high pole
            depositTangent = 180;

            ///////////////////////////////////////////
            //  CHANGE ONLY IF ABSOLUTELY NECESSARY  //
            //           DURING TOURNAMENT           //
            ///////////////////////////////////////////

            robot.turret.turret_PRELOAD_POSITION_VALUE  = -170;
            robot.turret.turret_PICKUP_POSITION_VALUE   = 0;
            robot.turret.turret_DEPOSIT_POSITION_VALUE  = 290;  //hitting hard stop

            robot.arm.EXTENSION_POSITION_PICKUP = 0;
            robot.arm.EXTENSION_POSITION_PRELOAD = 0.40;
            robot.arm.EXTENSION_POSITION_DEPOSIT = 0.40;

            ///////////////////////////////////////////
            //      MAKE ADJUSTMENTS ON POSES        //
            //           DURING TOURNAMENT           //
            ///////////////////////////////////////////

            preloadDeltaX = 2;
            preloadDeltaY = 0;

            pickupDeltaX = 0;
            pickupDeltaY = 0;

            depositDeltaX = 0;
            depositDeltaY = 0;
        }

        // Load the initial cone
        telemetry.clearAll();
        telemetry.addLine("Load Cone.  Driver 1 Hit A.");
        telemetry.update();

        while (!gamepad1.a && !isStopRequested()) {
        }

        // grab the preloaded cone
        robot.grabber.grabberClose();
        sleep(500);

        // this variable is used to calculate liftPositionPickup for stacked cones
        robot.lift.numCyclesCompleted = 0;
        robot.lift.updateLiftPickupPosition();

        // reset the encoders
        robot.lift.resetEncoders();
        robot.turret.resetEncoders();


        // Determine trajectory segment positions based on Alliance and Orientation
        startingPose = new Pose2d(XFORM_X * 36, XFORM_Y * 63, Math.toRadians(startingHeading));
        preloadPose = new Pose2d(XFORM_X * (16 + preloadDeltaX), XFORM_Y * (11.5 + preloadDeltaY), Math.toRadians(preloadHeading));

        // TODO: For BETA: Deposit position is middle of tile before uncontested high pole with robot angled 45deg.
        depositPose = new Pose2d(XFORM_X * (12 + depositDeltaX), XFORM_Y * (11.5 + depositDeltaY), Math.toRadians(depositHeading));

        // TODO: For BETA: Pickup stop position reduced by 5in which was transferred to .forward() function with reduced speed.
        pickupPose = new Pose2d(XFORM_X * (54-5 + pickupDeltaX), XFORM_Y * (11.5 + pickupDeltaY), Math.toRadians(pickupHeading));
        parkingPose = new Pose2d(); // to be defined after reading the signal cone

        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        // Build trajectory sequences before Start signal
        TrajectorySequence Trajectory = buildTrajectory(robot);
        Trajectory trajectoryPark;

        telemetry.clearAll();
        telemetry.addLine("Robot is Ready!");
        telemetry.update();

        /********************* Initialization Complete **********************/


        //-------------------------------------------------------------------
        // waitForStart();  // Replaced with the following while loop
        //-------------------------------------------------------------------

        //camera
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == RIGHT || tag.id == MIDDLE) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\n But we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        autoTime.reset();   // Start your autonomous counter

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            // Couldn't read the tag, choose whichever tag is harder to read
            PARKING_NUMBER = 2;
        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */
            // Assign PARKING_NUMBER based on tagOfInterest
            // Determine parking position based on Alliance and Orientation
            if (tagOfInterest.id == LEFT)
                PARKING_NUMBER = 1;
            else if (tagOfInterest.id == MIDDLE)
                PARKING_NUMBER = 2;
            else
                PARKING_NUMBER = 3;
        }

        if (isOrientationLEFT) {   // LEFT
            switch (PARKING_NUMBER) {
                case 1:
                    parkingPose = new Pose2d(-58.75, -11.75, Math.toRadians(180));
                    break;

                case 2:
                    parkingPose = new Pose2d(-35.25, -11.75, Math.toRadians(180));
                    break;

                case 3:
                    parkingPose = new Pose2d(-11.75, -11.75, Math.toRadians(180));
                    break;
            }
        }
        else {                     // RED-RIGHT
            switch (PARKING_NUMBER) {
                case 1:
                    parkingPose = new Pose2d(11.75, -11.75, Math.toRadians(0));
                    break;

                case 2:
                    parkingPose = new Pose2d(35.25, -11.75, Math.toRadians(0));
                    break;

                case 3:
                    parkingPose = new Pose2d(58.75, -11.75, Math.toRadians(0));
                    break;
            }
        }

        // initiate first trajectory asynchronous (go to pickup location) at the start of autonomous
        // Need to call drive.update() to make things move within the loop
        currentTrajectoryState = TrajectoryState.TRAJECTORY_RUNNING_STATE;
        robot.drive.followTrajectorySequenceAsync(Trajectory);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentTrajectoryState) {
                case TRAJECTORY_RUNNING_STATE:
                    if (!robot.drive.isBusy()) {
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_PARKING_STATE;
                    }
                    break;

                case TRAJECTORY_PARKING_STATE:
                    // TODO: Check if this passed value is still good. Note that the EXTENSION_POSITION_HOME value was changed in ExtensionA class
                    robot.arm.extendHome();
                    robot.alignment.alignUp();
                    robot.grabber.grabberClose();
                    robot.turret.moveTo(robot.turret.CENTER_POSITION_VALUE);

                    trajectoryPark = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(parkingPose)
                            .build();
                    robot.drive.followTrajectory(trajectoryPark); // This is synchronous trajectory; code does not advance until the trajectory is complete

//                    robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_RESET);

                    currentTrajectoryState = TrajectoryState.TRAJECTORY_IDLE;

                    break;

                case TRAJECTORY_IDLE:
                    // Lower the lift closer to the ground
                    // Do nothing. This concludes the autonomous program
                    break;
            }

            // Is it time to park?
            if (autoTime.seconds() > TIME_TO_PARK &&
                    (currentTrajectoryState != TrajectoryState.TRAJECTORY_PARKING_STATE) &&
                    currentTrajectoryState != TrajectoryState.TRAJECTORY_IDLE) {
                currentTrajectoryState = TrajectoryState.TRAJECTORY_PARKING_STATE;
            }
            else {

//                telemetry.addData("Heading=",Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
//                telemetry.addData("X=",robot.drive.getPoseEstimate().getX());
//                telemetry.addData("Y=",robot.drive.getPoseEstimate().getY());
//                telemetry.addData("Lift Position=",robot.lift.getPosition());
//                telemetry.addData("Current Turret Position=", robot.turret.getPosition());
//                telemetry.addData("Target Turret Position =", robot.turret.currentTargetPosition);

                telemetry.addData("Current Cycle: ", robot.lift.numCyclesCompleted);
                telemetry.addData("Pickup Height: ", robot.lift.liftPositionPickup);

                telemetry.update();

                // Continue executing trajectory following
                robot.drive.update();
                robot.turret.transitionToPosition();

                // Execute systems based on stateMap
//                robot.updateSystems();
            }

        }
    }


    //camera
    void tagToTelemetry(AprilTagDetection detection) {
        Ending_Location = detection.id;
        telemetry.addData("Thing :", Ending_Location);
        telemetry.update();
    }//camera

    private void setProgram() {
        orientationIsSet = false;

        telemetry.clearAll();
        telemetry.addLine("Set Orientation: Driver 1-> dpad left or right.");
        telemetry.update();

        while (!orientationIsSet && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                isOrientationLEFT = true;
                orientationIsSet = true;
            } else if (gamepad1.dpad_right) {
                isOrientationLEFT = false;
                orientationIsSet = true;
            }
        }

        String orientation;
        if (isOrientationLEFT) {
            orientation = "LEFT";
        } else {
            orientation = "RIGHT";
        }

        telemetry.clearAll();
        telemetry.addLine("Confirm Program:");
        telemetry.addData("Orientation:", orientation);
        telemetry.addLine("Driver 2-> A To Confirm. B to Restart.");
        telemetry.update();

        boolean confirmation = false;
        while (!confirmation && !isStopRequested()) {
            telemetry.clearAll();
            if (gamepad2.a) {
                telemetry.clearAll();
                telemetry.addLine("Program Confirmed");
                telemetry.update();
                programConfirmation = true;
                confirmation = true;
            } else if (gamepad2.b) {
                telemetry.clearAll();
                telemetry.addLine("Program Rejected");
                telemetry.update();
                programConfirmation = false;
                orientationIsSet = false;
                allianceIsSet = false;
                confirmation = true;
            }
        }

        sleep(500);
        telemetry.clearAll();
    }

}