package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: 22312Tele", group="Robot")
public class TestTeleOp extends LinearOpMode {
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


    private double extensionPosition = 0.025;

    private final String MANUAL_DRIVE_MODE = "MANUAL";
    private final String AUTO_DRIVE_MODE = "AUTO";
    private final String DRIVE_MODE = "DRIVE_MODE";

    private boolean leftTriggerPressed = false;
    private final double SLOWMODE  = 0.45;

    Constants constants = new Constants();



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
    }};

    public void runOpMode() {

        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
//        robot.initializeRobotPosition();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_HIGH);
        stateMap.put(constants.DRIVER_2_SELECTED_TURRET, robot.turret.CENTER_POSITION);

        stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE);


        waitForStart();
      while (opModeIsActive()) {
      if(gamepad2.right_trigger > 0.1){
          robot.lift.setMotor(1.0);
      } else if(gamepad2.left_trigger > 0.1) {
          robot.lift.setMotor(-1.0);
      } else if(gamepad2.left_bumper) {
          robot.lift.resetEncoders();
      } else {
        setButtons();

        if (toggleMap.get(GAMEPAD_1_B_STATE)) {
            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
        } else {
            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        }

        if(toggleMap.get(GAMEPAD_1_A_STATE)){
            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(constants.DRIVER_2_SELECTED_LIFT));
        } else {
            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        }

        if(gamepad2.a){
            stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_LOW);
        } else if(gamepad2.b){
            stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_MEDIUM);
        } else if(gamepad2.y){
            stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_HIGH);
        }

        if(gamepad2.dpad_right){
            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
        } else if(gamepad2.dpad_left){
            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
        }else if(gamepad2.dpad_up){
            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        }
        while(gamepad2.right_stick_y > 0.2){
            robot.arm.joyStickExtension(extensionPosition);
        }
        while(gamepad2.right_stick_y < -0.2){
            robot.arm.joyStickExtension((-extensionPosition));
        }
        if(gamepad1.right_trigger > 0.5 && stateMap.get(constants.CONE_CYCLE).equalsIgnoreCase(constants.STATE_NOT_STARTED)){
            stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
        }

        if (gamepad1.dpad_down) {
            stateMap.put(DRIVE_MODE, AUTO_DRIVE_MODE);
            toggleMap.put(GAMEPAD_1_A_STATE, false);
            TrajectorySequence reverseTrajectory = drive.highSpeedTrajectoryBuilder(drive.getPoseEstimate())
                    .back(40)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND))
                    .addDisplacementMarker(() -> stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE))
                    .build();

            drive.followTrajectorySequenceAsync(reverseTrajectory);

        } else if (gamepad1.dpad_up) {
            stateMap.put(DRIVE_MODE, AUTO_DRIVE_MODE);
            toggleMap.put(GAMEPAD_1_A_STATE, true);
            TrajectorySequence forwardTrajectory = drive.highSpeedTrajectoryBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH))
                    .forward(42)
                    .addDisplacementMarker(() -> stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE))
                    .build();

            drive.followTrajectorySequenceAsync(forwardTrajectory);

        }
        if(gamepad2.left_stick_x <= -0.1 || gamepad2.left_stick_x >= 0.1){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            -(0.5 * gamepad1.left_stick_x),
                            0
                    )
            );

        }

        if (stateMap.get(DRIVE_MODE).equals(MANUAL_DRIVE_MODE)) {
              if (toggleMap.get(GAMEPAD_1_LEFT_TRIGGER_STATE)) {
                  drive.setWeightedDrivePower(
                          new Pose2d(

                                  (SLOWMODE * -gamepad1.left_stick_y),
                                  (SLOWMODE * -gamepad1.left_stick_x),
                                  (SLOWMODE * -gamepad1.right_stick_x)
                          )
                  );
              } else {
                  drive.setWeightedDrivePower(
                          new Pose2d(

                                  -gamepad1.left_stick_y,
                                  -gamepad1.left_stick_x,
                                  -gamepad1.right_stick_x
                          )
                  );
              }
          }

        drive.update();

        robot.updateSystems();

        telemetry.update();
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
