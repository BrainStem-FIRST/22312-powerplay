package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.Map;
import java.util.Timer;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;


@Config
@Autonomous(name="Robot: Test Auto Synchronous", group="Robot")
public class Auto extends LinearOpMode {

    public static int PARKING_NUMBER = 1; // Controlled by the dashboard for test purposes
    public static double SPEED = 50.0;    // Controlled by the dashboard for test purposes

    // Alliance variables
    public static boolean isAllianceRED = true;
    public static boolean isOrientationLEFT = true;

    private ElapsedTime autoTime = new ElapsedTime();
    private double TIME_TO_PARK = 25.0;

    Constants constants = new Constants();



    @Override
    public void runOpMode() {

        boolean inParkingSequence = false;
        int     numCyclesCompleted = 0;

        //may or may not put state maps here
        Map<String, String> stateMap = new HashMap<String, String>() {};

        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);

        /*************** Initialize the robot *****************/

        // TODO: drive is already an object within BrainStemRobot
        // SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        /*************** Declare variable used for Autonomous *******************/

        // For initial testing purposes, assume the robot is in RED ALLIANCE and
        // the starting position is on LEFT
        //
        // Need a variable to determine the association with alliance and
        // adjust the coordinate system accordingly.

        Pose2d      startingPose, pickupPose, deliverPose, parkingPose = null;
        Trajectory  trajectory1, trajectory2, trajectory3, trajectoryPark;

        // Determine waypoints based on Alliance and Orientation
        if(isAllianceRED) {
            if(isOrientationLEFT) { // RED-LEFT
                startingPose    = new Pose2d(-36, -67.25, Math.toRadians(90));
                pickupPose      = new Pose2d(-64.25, -16, Math.toRadians(180));
                deliverPose     = new Pose2d(-24, -16, Math.toRadians(180));
            }
            else {                  // RED-RIGHT
                startingPose    = new Pose2d(36, -67.25, Math.toRadians(90));
                pickupPose      = new Pose2d(64.25, -16, Math.toRadians(0));
                deliverPose     = new Pose2d(24, -16, Math.toRadians(0));
            }
        }
        else {
            if(isOrientationLEFT) { // BLUE-LEFT
                startingPose    = new Pose2d(36, 67.25, Math.toRadians(-90));
                pickupPose      = new Pose2d(64.25, 16, Math.toRadians(0));
                deliverPose     = new Pose2d(24, 16, Math.toRadians(0));
            }
            else {                  // BLUE-RIGHT
                startingPose    = new Pose2d(-36, 67.25, Math.toRadians(-90));
                pickupPose      = new Pose2d(-64.25, 16, Math.toRadians(180));
                deliverPose     = new Pose2d(-24, 16, Math.toRadians(180));
            }
        }

        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        // Determine parking position based on Alliance and Orientation
        if(isAllianceRED) {
            if(isOrientationLEFT)   // RED-LEFT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (-60, -12, Math.toRadians(180));
                        break;

                    case 2:
                        parkingPose = new Pose2d (-36, -12, Math.toRadians(180));
                        break;

                    case 3:
                        parkingPose = new Pose2d (-12, -12, Math.toRadians(-90));
                        break;
                }
            else                    // RED-RIGHT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (12, -12, Math.toRadians(-90));
                        break;

                    case 2:
                        parkingPose = new Pose2d (36, -12, Math.toRadians(0));
                        break;

                    case 3:
                        parkingPose = new Pose2d (60, -12, Math.toRadians(0));
                        break;
                }
        }
        else {
            if(isOrientationLEFT)   // BLUE-LEFT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (60, 12, Math.toRadians(0));
                        break;

                    case 2:
                        parkingPose = new Pose2d (36, 12, Math.toRadians(0));
                        break;

                    case 3:
                        parkingPose = new Pose2d (12, 12, Math.toRadians(90));
                        break;
                }
            else                    // BLUE-RIGHT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (-12, 12, Math.toRadians(90));
                        break;

                    case 2:
                        parkingPose = new Pose2d (-36, 12, Math.toRadians(-90));
                        break;

                    case 3:
                        parkingPose = new Pose2d (-60, 12, Math.toRadians(-90));
                        break;
                }
        }

        /****************  define trajectories  *********************/

        // From starting position, move forward and spline to pickup position
        trajectory1 = robot.drive.trajectoryBuilder(startingPose)
                //moves forward in a line by 54 in and faces 90 degrees away by end
                .forward(43.25)
                .splineTo(new Vector2d(pickupPose.getX(), pickupPose.getY()), pickupPose.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        // starting from where trajectory1 ended up, assume cone is grabbed, move back to depositing position
        trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end(), true)
                //moves backwards to the delivery position
                .lineToLinearHeading(deliverPose,
                        SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        // starting from where trajectory2 ended up, assume cone was delivered, move back to pickup position
        trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end(), false)
                //moves forwards to the pickup position
                .splineTo(new Vector2d(pickupPose.getX(), pickupPose.getY()), pickupPose.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        /********************* Initialization Complete **********************/

        waitForStart();

        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        robot.updateSystems();

        // go to pickup location at the start of autonomous
        // DOES THIS HAVE TO BE INSIDE THE WHILE LOOP OR BOUNDED BY isStoppedRequested()?
        robot.drive.followTrajectory(trajectory1);

        while (!isStopRequested() && !inParkingSequence) {
            // Repeat pickup-delivery cycle as many as you can

            // TODO: Grab the cone, lift, etc. Need to set state and update once merged to working Main branch
            stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
            while(!stateMap.get(constants.CONE_CYCLE).equalsIgnoreCase(constants.STATE_NOT_STARTED)) {
                robot.updateSystems();
            }


            // go to delivery position
            robot.drive.followTrajectory(trajectory2);

            // engage cone depositing sequence
            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
            robot.updateSystems();

            stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
            while(!stateMap.get(constants.CONE_CYCLE).equalsIgnoreCase(constants.STATE_NOT_STARTED)) {
                robot.updateSystems();
            }

            // go back to pickup position
            robot.drive.followTrajectory(trajectory3);

            // Is it time to park?
            if (autoTime.seconds() > TIME_TO_PARK)
                inParkingSequence = true;

            numCyclesCompleted += 1;
            telemetry.addData("Cycle:", "%d", numCyclesCompleted);
            telemetry.update();
        }

        if (!isStopRequested()) {
            // Build parking trajectory from where ever the robot stopped at the end of timer
            // (if things are synchronous, the starting pose of Parking trajectory is known and
            // the trajectory can be built ahead of time)
             trajectoryPark = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .splineTo(new Vector2d(parkingPose.getX(), parkingPose.getY()), parkingPose.getHeading(),
                            SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
            robot.drive.followTrajectory(trajectoryPark);
        }
    }
}