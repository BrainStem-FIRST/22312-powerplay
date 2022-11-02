package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Auto {
    private Pose2d startPosition = new Pose2d(); //TODO: set x and y
    SampleMecanumDrive drive = new SampleMecanumDrive(opMode.hardwareMap);

    drive.setPoseEstimate(startPosition);

    Trajectory depositPreLoad = drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToLinearHeading()
            .build();
    drive.followTrajectory(depositPreLoad);
}
//this is very basic trajectory code to get the general gist of the concept--NOT COMPLETE!