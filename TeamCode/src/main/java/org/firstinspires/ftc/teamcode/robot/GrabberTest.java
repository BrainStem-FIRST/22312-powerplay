package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.autoclasses.BrainStemRobotA;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: Grabber Test", group="Robot")
public class GrabberTest extends LinearOpMode{

    public void runOpMode() {
        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainStemRobotA robot = new BrainStemRobotA(hardwareMap, telemetry, stateMap);
        robot.lift.resetEncoders();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.b){
                robot.arm.extendMax();
            }
            if(gamepad1.x){
                robot.arm.extendHome();
            }
            if(gamepad1.right_trigger > 0.2){
                robot.grabber.grabberClose();
            }
            if(gamepad1.left_trigger > 0.2){
                robot.grabber.grabberOpen();
            }
            if(gamepad1.a){
                robot.grabber.grabberOpenWide();
            }
            telemetry.addData("lift position:", robot.lift.getPosition());
            telemetry.update();
        }
    }
}