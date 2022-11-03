package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        Pose2d startPosition = new Pose2d(20, 20);

        Map<String, String> stateMap = new HashMap<String, String>() {{
        }};
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
        //robot.initializeRobotPosition();

        //may or may not put state maps here

        //define trajectories
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                //moves in a line to (40, 40) and faces 90 degrees away by end
                .lineToLinearHeading(new Pose2d(40, 20, Math.toRadians(90)))
                .build();

        waitForStart();

        while (!isStopRequested()) {
            robot.drive.followTrajectory(trajectory1);
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