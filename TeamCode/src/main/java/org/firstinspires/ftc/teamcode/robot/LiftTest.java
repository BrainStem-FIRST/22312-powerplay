package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;


@TeleOp(name="Robot: LIFTCLASS", group="Robot")
public class LiftTest extends LinearOpMode {
    private final String GAMEPAD_1_A_STATE = "GAMEPAD_1_A_STATE";
    private final String GAMEPAD_1_A_IS_PRESSED = "GAMEPAD_1_A_IS_PRESSED";
    private final String GAMEPAD_1_B_STATE = "GAMEPAD_1_B_STATE";
    private final String GAMEPAD_1_B_IS_PRESSED = "GAMEPAD_1_B_IS_PRESSED";
    private final String GAMEPAD_1_X_STATE = "GAMEPAD_1_X_STATE";
    private final String GAMEPAD_1_X_IS_PRESSED = "GAMEPAD_1_X_IS_PRESSED";
    private final String GAMEPAD_1_RIGHT_STICK_PRESSED = "GAMEPAD_1_RIGHT_STICK_PRESSED ";
    private final String GAMEPAD_1_RIGHT_STICK_STATE = "GAMEPAD_1_RIGHT_STICK";
    private final String GAMEPAD_1_LEFT_STICK_PRESSED = "GAMEPAD_1_LEFT_STICK_PRESSED";
    private final String GAMEPAD_1_LEFT_STICK_STATE =  "GAMEPAD_1_LEFT_STICK_STATE";
    private final String GAMEPAD_1_LEFT_TRIGGER_STATE  = "GAMEPAD_1_LEFT_TRIGGER_STATE";
    private final String GAMEPAD_1_LEFT_TRIGGER_PRESSED = "GAMEPAD_1_LEFT_TRIGGER_PRESSED";
    private final String GAMEPAD_1_Y_STATE = "GAMEPAD_1_Y_STATE";
    private final String GAMEPAD_1_Y_PRESSED = "GAMEPAD_1_Y_IS_PRESSED";
    private final String GAMEPAD_1_LEFT_BUTTON_STATE = "GAMEPAD_1_LEFT_BUTTON_STATE";
    private final String GAMEPAD_1_LEFT_BUTTON_PRESSED = "GAMEPAD_1_LEFT_BUTTON_PRESSED";

    private ElapsedTime elapsedTime = new ElapsedTime();


    private double extensionPosition = 0.01;
    private int GROUND_LIFT_FINE_ADJUSTMENTS = 1;

    private final String MANUAL_DRIVE_MODE = "MANUAL";
    private final String AUTO_DRIVE_MODE = "AUTO";
    private final String DRIVE_MODE = "DRIVE_MODE";
    private final int checkTicks = 10;
    private final int extensionAddition = 6;

    private int CONE_COUNT = 1;

    private boolean leftTriggerPressed = false;
    private boolean retractionInProgress = false;
    private final double SLOWMODE  = 0.45;
    private boolean CAP_MODE = false;

    Constants constants = new Constants();

    private ElapsedTime grabberCapCycleTime = new ElapsedTime();
    private ElapsedTime grabberCycleTime = new ElapsedTime();
    private boolean grabberCycleInProgress = false;
    private boolean grabberyCapCycleInProgress = false;

    private double k_regularTurningSpeed = 0.4;


    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
        put(GAMEPAD_1_A_STATE, false);
        put(GAMEPAD_1_A_IS_PRESSED, false);
        put(GAMEPAD_1_B_STATE, false);
        put(GAMEPAD_1_B_IS_PRESSED, false);
        put(GAMEPAD_1_X_STATE, false);
        put(GAMEPAD_1_X_IS_PRESSED, false);
        put(GAMEPAD_1_RIGHT_STICK_STATE, false);
        put(GAMEPAD_1_RIGHT_STICK_PRESSED, false);
        put(GAMEPAD_1_LEFT_STICK_PRESSED, false);
        put(GAMEPAD_1_LEFT_TRIGGER_STATE ,false);
        put(GAMEPAD_1_LEFT_TRIGGER_PRESSED, false);
        put(GAMEPAD_1_Y_STATE, false);
        put(GAMEPAD_1_Y_PRESSED, false);
        put(GAMEPAD_1_LEFT_BUTTON_STATE, false);
        put(GAMEPAD_1_LEFT_BUTTON_PRESSED, false);
    }};

    public void runOpMode() {

        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
//        robot.initializeRobotPosition();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
//        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
//        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
//        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
//        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_HIGH);
        stateMap.put(constants.DRIVER_2_SELECTED_TURRET, robot.turret.CENTER_POSITION);

        stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE);


        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.liftMotor2.setPower(1.0);
                robot.lift.liftMotor.setPower(1.0);
            }
            if(gamepad1.b){
                robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.liftMotor2.setPower(-1.0);
                robot.lift.liftMotor.setPower(-1.0);
            }
        }
    }


    private void setButtons() {
        toggleButton(GAMEPAD_1_A_STATE, GAMEPAD_1_A_IS_PRESSED, gamepad1.a);
        toggleButton(GAMEPAD_1_B_STATE, GAMEPAD_1_B_IS_PRESSED, gamepad1.b);
        toggleButton(GAMEPAD_1_X_STATE, GAMEPAD_1_X_IS_PRESSED, gamepad1.x);
//        toggleButton(GAMEPAD_1_RIGHT_STICK_STATE, GAMEPAD_1_RIGHT_STICK_PRESSED, gamepad1.right_stick_button);
//        toggleButton(GAMEPAD_1_LEFT_STICK_STATE, GAMEPAD_1_LEFT_STICK_PRESSED, gamepad1.left_stick_button);
        toggleButton(GAMEPAD_1_LEFT_TRIGGER_STATE, GAMEPAD_1_LEFT_STICK_PRESSED,gamepad1.left_trigger >= 0.5);
        toggleButton(GAMEPAD_1_Y_STATE, GAMEPAD_1_Y_PRESSED,gamepad1.y);
        toggleButton(GAMEPAD_1_LEFT_BUTTON_STATE, GAMEPAD_1_LEFT_BUTTON_PRESSED, gamepad1.left_bumper);
    }


    private boolean toggleButton(String buttonStateName, String buttonPressName, boolean buttonState) {
        boolean buttonPressed = toggleMap.get(buttonPressName);
        boolean toggle = toggleMap.get(buttonStateName);

        if (buttonState) {
            if (!buttonPressed) {
                toggleMap.put(buttonStateName, !toggle);
                toggleMap.put(buttonPressName, true);
            }
        } else {
            toggleMap.put(buttonPressName, false);
        }

        return toggleMap.get(buttonStateName);
    }



}

