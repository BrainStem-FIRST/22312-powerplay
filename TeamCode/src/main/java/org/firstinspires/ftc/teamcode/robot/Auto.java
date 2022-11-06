package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    // Test different trajectories by changing TEST_NUM from the dashboard
    // 1: Synchronous
    // 2: Speed set
    public static int TEST_NUMBER = 2;
    public static int PARKING_NUMBER = 1;
    public static double SPEED = 50.0;    // slower speed, in/sec
    public static int PARKING_POSITION = 1;

    @Override
    public void runOpMode() {

        boolean runTest = false;

        //may or may not put state maps here
        Map<String, String> stateMap = new HashMap<String, String>() {};

        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);

        // Initialize the robot

        // TODO: drive is already an object within BrainStemRobot
        // SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        // For initial testing purposes, assume the robot is in RED ALLIANCE and
        // the starting position is on LEFT
        //
        // Need a variable to determine the association with alliance and
        // adjust the coordinate system accordingly.
        Trajectory trajectory1, trajectory2;

        Pose2d startingPose = new Pose2d(-36,-67.25,Math.toRadians(90)); // On Red Wall facing Blue Wall
        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        //define trajectories
        switch(TEST_NUMBER) {
            case 1: // Regular speed
                // From starting position of RED-LEFT; relative movements to starting position
                trajectory1 = robot.drive.trajectoryBuilder(startingPose)
                        //moves forward in a line by 40 in and faces 90 degrees away by end
                        .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(90)))
                        //turns left and ends up 40 inches to the right, heading facing cones
                        .splineTo(new Vector2d(-24, 48), Math.toRadians(180))
                        .build();

                // starting from where trajectory1 ended up, assume cone is grabbed, move back to depositing position
                trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end(), true)
                        //moves backwards in a line by 40 inches on x-axis; still faces the cones
                        .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(0)))
                        .build();

                runTest = true;
                telemetry.addData("Test #", "%d", TEST_NUMBER);
                telemetry.update();

                break;

            case 2: // Same as 1 with slower speed set at Dashboard
                // From starting position of RED-LEFT; relative movements to starting position
                trajectory1 = robot.drive.trajectoryBuilder(startingPose)
                        //moves forward in a line by 40 in and faces 90 degrees away by end
                        .splineTo(new Vector2d(-36, -24), Math.toRadians(90),
                                SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .splineTo(new Vector2d(-54, -16), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .splineTo(new Vector2d(-64.25, -16), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                        /* .lineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        //turns left and ends up 40 inches to the right, heading facing cones
                        .splineTo(new Vector2d(-64.25, -14), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL)) */

                // starting from where trajectory1 ended up, assume cone is grabbed, move back to depositing position
                trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end(), true)
                        //moves backwards in a line by 40 inches on x-axis; still faces the cones
                        .lineToLinearHeading(new Pose2d(-24, -14, Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .build();

                runTest = true;
                telemetry.addData("Test #", "%d", TEST_NUMBER);
                telemetry.update();
                break;

            default:
                trajectory1 = robot.drive.trajectoryBuilder(startingPose)
                        .build();
                trajectory2 = robot.drive.trajectoryBuilder(startingPose)
                        .build();

                runTest = false;
                telemetry.addData("Test #", "INVALID");
                telemetry.update();
        }

        waitForStart();

        while (!isStopRequested() && runTest) {
            robot.drive.followTrajectory(trajectory1);

            // TODO: Grab the cone, lift, etc. Need to set state and update once merged to working Main branch
            // engage cone grabbing sequence
            /* stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
            robot.updateSystems(); */
            sleep(1000);

            robot.drive.followTrajectory(trajectory2);

            // engage cone depositing sequence
            /* stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.PLACEMENT_HEIGHT);
            robot.updateSystems(); */
            sleep (1000);

            runTest = false;

            /********************************
            if (elapsedTime > timeToPark){
                // stop trajectory
                // determine robot position
                // switch statement
                // for parking lot one, x position is between minus something to minus something
                // for parking lot two, the same but the somethings are different values
                // for parking lot three, same as parking lot two
                // TODO: figure out values for parking lots
            }
            ********************************/
            
            Vector2d parkingPosition = new Vector2d();
            Timer elapsedTime = new Timer();
            double timeToPark = 25;

            if (elapsedTime > timeToPark)
                switch (PARKING_NUMBER){
                    case 1:
                        parkingPosition = new Vector2d (-60, -36);

                    case 2:
                        parkingPosition = new Vector2d (-36, -36);

                    case 3:
                        parkingPosition = new Vector2d (-12, -36);
                }

            Trajectory trajectory3 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .splineTo(parkingPosition, Math.toRadians(90))
                    .build();
        }
    }
}