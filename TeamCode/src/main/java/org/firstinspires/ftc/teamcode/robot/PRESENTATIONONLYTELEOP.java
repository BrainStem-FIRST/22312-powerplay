package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;


@TeleOp(name="Robot: TELEPRESENTATION", group="Robot")
public class PRESENTATIONONLYTELEOP extends LinearOpMode {

    public void runOpMode() {
        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
        robot.turret.resetEncoders();
        robot.lift.resetEncoders();

        robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_right){
                robot.turret.turretMotor.setTargetPosition(robot.turret.RIGHT_POSITION_VALUE);
                robot.turret.turretMotor.setPower(1.0);
            }
            if(gamepad1.dpad_down){
                robot.turret.turretMotor.setTargetPosition(robot.turret.CENTER_POSITION_VALUE);
                robot.turret.turretMotor.setPower(1.0);
            }
            if(gamepad1.dpad_left){
                robot.turret.turretMotor.setTargetPosition(robot.turret.LEFT_POSITION_VALUE);
                robot.turret.turretMotor.setPower(1.0);
            }
            if(gamepad1.a){
                robot.lift.liftMotor.setTargetPosition(robot.lift.LIFT_POSITION_LOWPOLE);
                robot.lift.liftMotor2.setTargetPosition(robot.lift.LIFT_POSITION_LOWPOLE);
                robot.lift.liftMotor.setPower(1.0);
                robot.lift.liftMotor2.setPower(1.0);

                telemetry.addData("Target: ", robot.lift.LIFT_POSITION_LOWPOLE);
            }
            if(gamepad1.b){
                robot.lift.liftMotor.setTargetPosition(robot.lift.LIFT_POSITION_MIDPOLE);
                robot.lift.liftMotor2.setTargetPosition(robot.lift.LIFT_POSITION_MIDPOLE);
                robot.lift.liftMotor.setPower(1.0);
                robot.lift.liftMotor2.setPower(1.0);

                telemetry.addData("Target: ", robot.lift.LIFT_POSITION_MIDPOLE);
            }
            if(gamepad1.y){
                robot.lift.liftMotor.setTargetPosition(robot.lift.LIFT_POSITION_HIGHPOLE);
                robot.lift.liftMotor2.setTargetPosition(robot.lift.LIFT_POSITION_HIGHPOLE);
                robot.lift.liftMotor.setPower(1.0);
                robot.lift.liftMotor2.setPower(1.0);

                telemetry.addData("Target: ", robot.lift.LIFT_POSITION_HIGHPOLE);
            }
            if(gamepad1.right_bumper){
                robot.flippers.bothFlippersDown();
            }
            if(gamepad1.left_bumper){
                robot.flippers.bothFlippersUp();
            }
            telemetry.addData("LiftMotor encoder:  ", robot.lift.liftMotor.getCurrentPosition());
            telemetry.addData("LiftMotor2 encoder: ", robot.lift.liftMotor2.getCurrentPosition());
            telemetry.update();

            if(gamepad1.right_trigger > 0.2){
                robot.grabber.grabberClose();
            }
            if(gamepad1.x){ // it was gamepad1.b before which is reserved for lift position midpole; changed to .x
                robot.arm.extendMax();
            }
            }
        }
    }