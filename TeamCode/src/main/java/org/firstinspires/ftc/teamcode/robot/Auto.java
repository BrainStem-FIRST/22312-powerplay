package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.Map;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class Auto extends LinearOpMode {

    public void runOpMode() {

        //may or may not put state maps here
        Map<String, String> stateMap = new HashMap<String, String>() {};

        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
        //robot.initializeRobotPosition();

        // For initial testing purposes, assume the robot is in RED ALLIANCE and
        // the starting position is on LEFT
        //
        // Need a variable to determine the association with alliance and
        // adjust the coordinate system accordingly.
        Pose2d startingPose = new Pose2d(0,0,Math.toRadians(90)); // On Red Wall facing Blue Wall
        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        //define trajectories


        // From starting position of RED-LEFT; relative movements to starting position
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startingPose)
                //moves forward in a line by 40 in and faces 90 degrees away by end
                .lineToLinearHeading(new Pose2d(0, 40, Math.toRadians(90)))
                //turns left and ends up 40 inches to the right, heading facing cones
                .splineTo(new Vector2d(-40,40),Math.toRadians(180))
                .build();

        // starting from where trajectory1 ended up, assume cone is grabbed, move back to depositing position
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end(),true)
                //moves backwards in a line by 40 inches on x-axis; still faces the cones
                .lineToLinearHeading(new Pose2d(0, 40, Math.toRadians(0)))
                .build();

        waitForStart();

        while (!isStopRequested()) {
            robot.drive.followTrajectory(trajectory1);

            // TODO: Grab the cone, lift, etc. Need to set state and update once merged to working Main branch
            // engage cone grabbing sequence
            sleep(1000);    // remove after replacing with useful tasks

            robot.drive.followTrajectory(trajectory2);

            // engage cone depositing sequence
            sleep(1000);
        }
    }
}

/* FOCUSING ON THE CODE ABOVE
    drive.setPoseEstimate(startPosition);
    Trajectory depositPreLoad = drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToLinearHeading()
            .build();
    drive.followTrajectory(depositPreLoad);
}
this is very basic trajectory code to get the general gist of the concept--NOT COMPLETE!
 */