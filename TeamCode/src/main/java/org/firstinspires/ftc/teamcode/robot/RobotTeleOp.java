package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.Map;


@TeleOp(name="Robot: 22312Tele", group="Robot")
public class RobotTeleOp extends LinearOpMode {
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
    private final double extensionAddition = 0.035;

    private int CONE_COUNT = 1;
    public boolean extensionDelay = false;
    public int numberOfAClicks = 0;
    public ElapsedTime liftUp = new ElapsedTime();

    private boolean leftTriggerPressed = false;
    private boolean retractionInProgress = false;
    private final double TRANSLATIONAL_SLOWMODE_SPEED  = 0.35;
    private final double TURNING_SLOW_MODE = 0.4;
    private double REGULAR_TURNING_SPEED = 0.6;
    private double FLIPPERS_TURN_SPEED = 0.3;
    private boolean CAP_MODE = false;

    Constants constants = new Constants();

    private ElapsedTime grabberCapCycleTime = new ElapsedTime();
    private ElapsedTime grabberCycleTime = new ElapsedTime();
    private boolean grabberCycleInProgress = false;
    private boolean grabberyCapCycleInProgress = false;
    private ElapsedTime putDownFast = new ElapsedTime();
    private boolean fastDown = false;



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
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        stateMap.put(robot.flippers.SYSTEM_NAME, robot.flippers.FLIPPERS_UP);

        stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_HIGH);
        stateMap.put(constants.DRIVER_2_SELECTED_TURRET, robot.turret.CENTER_POSITION);

        stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE);


        waitForStart();
      while (opModeIsActive()) {
          if (gamepad2.right_bumper && gamepad2.left_bumper) {
              robot.lift.resetEncoders();
          } else if (gamepad2.left_trigger > 0.1) {
              robot.lift.setMotor(-1);
          } else if (gamepad2.x) {
              robot.turret.resetEncoders();
          } else {
              setButtons();
              telemetry.addData("State Map", stateMap);
              if (toggleMap.get(GAMEPAD_1_B_STATE)) {
                  stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
              } else {
                  stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
              }
              if(gamepad1.a){
                  liftUp.reset();
                  liftUp.startTime();
              }

              if (toggleMap.get(GAMEPAD_1_A_STATE)) {
//                  extensionDelay = true;
//                  liftUp.reset();
//                  liftUp.startTime();
                  if(liftUp.seconds() > 0.3 && liftUp.seconds() < 0.5){
                      toggleMap.put(GAMEPAD_1_B_STATE, true);
                  }
                  robot.lift.liftPickup = 0;
                  robot.lift.LIFT_POSITION_GROUND = 0;
                  telemetry.addData("Lift pickup", robot.lift.liftPickup);
                  stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(constants.DRIVER_2_SELECTED_LIFT));
                  stateMap.put(robot.turret.SYSTEM_NAME, stateMap.get(constants.DRIVER_2_SELECTED_TURRET));
                  CAP_MODE = false;
//                  if(numberOfAClicks == 1) {
//                      extensionDelay = true;
//                      liftUp.reset();
//                      liftUp.startTime();
//                  } else if(numberOfAClicks == 2){
//                      extensionDelay = false;
//                      numberOfAClicks = 0;
//                  }
              } else {
                  stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
                  robot.arm.adjustmentPosition = 0;
                  robot.arm.alignUp();
              }

              if(extensionDelay){
                  if(liftUp.seconds() > 0.5 && !CAP_MODE){
                      toggleMap.put(GAMEPAD_1_B_STATE, true);
                  }
              }

              if (gamepad2.a) {
                  stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_LOW);
              } else if (gamepad2.b) {
                  stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_MEDIUM);
              } else if (gamepad2.y) {
                  stateMap.put(constants.DRIVER_2_SELECTED_LIFT, robot.lift.LIFT_POLE_HIGH);
              }
              if(toggleMap.get(GAMEPAD_1_LEFT_BUTTON_STATE)){
                  stateMap.put(robot.flippers.SYSTEM_NAME, robot.flippers.FLIPPERS_DOWN);
              }else{
                  stateMap.put(robot.flippers.SYSTEM_NAME, robot.flippers.FLIPPERS_UP);
              }
              if (gamepad1.right_bumper) {
                  telemetry.addData("time", elapsedTime);
                 robot.lift.liftPickup = 0;
              }
              if (retractionInProgress) {
                  if (elapsedTime.seconds() > 0.1) {
                      stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                      stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_COMPLETELY);
                      stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.ALIGN_UP);
                  }
                  if (elapsedTime.seconds() > 0.6) {
//                stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                      toggleMap.put(GAMEPAD_1_A_STATE, false);
                      telemetry.addData("timer in ", true);
                      retractionInProgress = false;
                      elapsedTime.reset();
                  }
              }

              if(gamepad1.left_trigger > 0.2){
                  CAP_MODE = true;
              }

              if (gamepad2.dpad_right) {
                  stateMap.put(constants.DRIVER_2_SELECTED_TURRET, robot.turret.RIGHT_POSITION);
              } else if (gamepad2.dpad_left) {
                  stateMap.put(constants.DRIVER_2_SELECTED_TURRET, robot.turret.LEFT_POSITION);
              } else if (gamepad2.dpad_down) {
                  stateMap.put(constants.DRIVER_2_SELECTED_TURRET, robot.turret.CENTER_POSITION);
              }

              if(gamepad2.right_trigger > 0.2){
                  if(stateMap.get(robot.lift.LIFT_SYSTEM_NAME).equals(robot.lift.LIFT_POLE_GROUND)) {
                      telemetry.addLine("in the lift ground fine adjustment");
                      telemetry.update();
                      robot.lift.LIFT_POSITION_GROUND += 1;
                  } else if(stateMap.get(robot.lift.LIFT_SYSTEM_NAME).equals(robot.lift.LIFT_POLE_LOW)){
                      telemetry.addLine("in the lift pole low fine adjustment");
                      telemetry.update();
                      robot.lift.LIFT_POSITION_LOWPOLE += 1;
                  } else if(stateMap.get(robot.lift.LIFT_SYSTEM_NAME).equals(robot.lift.LIFT_POLE_MEDIUM)){
                      telemetry.addLine("in the lift pole medium fine adjustment");
                      telemetry.update();
                      robot.lift.LIFT_POSITION_MIDPOLE += 1;
                  } else {
                      telemetry.addLine("in the lift pole high fine adjustment");
                      telemetry.update();
                      robot.lift.LIFT_POSITION_HIGHPOLE += 1;
                  }
              }
              if (stateMap.get(robot.lift.LIFT_SYSTEM_NAME) != robot.lift.LIFT_POLE_GROUND) {
                  if (gamepad1.right_trigger > 0.05 && gamepad1.right_trigger < 0.9) {
                      robot.lift.liftMotor.setPower(0);
                      robot.lift.liftMotor2.setPower(0);
                      robot.lift.setAdjustmentHeight(gamepad1.right_trigger);
              } else if (gamepad1.right_trigger >= 0.9) {
                      elapsedTime.reset();
                      elapsedTime.startTime();
                      retractionInProgress = true;
                      stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
                      toggleMap.put(GAMEPAD_1_B_STATE, false);
                  } else {
                      robot.lift.setAdjustmentHeight(0);
                  }
              } else if (gamepad1.right_trigger > 0.5 && !CAP_MODE) {
                  grabberCycleTime.reset();
                  grabberCycleTime.startTime();
                  stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                  grabberCycleInProgress = true;
//              stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
              } else if(gamepad1.right_trigger > 0.5 && CAP_MODE){
                  grabberCycleTime.reset();
                  grabberCycleTime.startTime();
                  stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                  grabberyCapCycleInProgress = true;
              }
//          telemetry.addData("Time",grabberCycleTime.milliseconds());
//          telemetry.addData("grabberCycleInProgress", grabberCycleInProgress);

              if (grabberCycleInProgress) {
                  if(grabberCycleTime.milliseconds() >= 150 && grabberCycleTime.milliseconds() <= 350){
                      robot.lift.setRawPower(1.0);
                      telemetry.addData("Giving jump", true);
                  }
                  if (grabberCycleTime.milliseconds() > 300 && robot.lift.LIFT_POSITION_GROUND == 0) {
                      robot.lift.liftPickup = 100;
                      grabberCycleInProgress = false;
                  } else if(grabberCycleTime.milliseconds() > 300 && robot.lift.LIFT_POSITION_GROUND != 0){
                      robot.lift.liftPickup = 100;
                      grabberCycleInProgress = false;
                  }
              }

              if(grabberyCapCycleInProgress){
                  if(grabberCapCycleTime.milliseconds() > 200){
                      robot.lift.liftPickup = 30;
                      grabberyCapCycleInProgress = false;
                  }
              }

//              if(toggleMap.get(GAMEPAD_1_LEFT_BUTTON_STATE)){
//                  robot.flippers.bothFlippersDown();
//              } else{
//                  robot.flippers.bothFlippersDown();
//              }
              if(gamepad1.left_trigger > 0.7){
                  robot.lift.liftPositionPickup = 0;
                  telemetry.addData("Robot lift pickup", robot.lift.liftPositionPickup);
              }
//        if(gamepad1.left_trigger > 0.2){
//            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_CHECK_STATE);
//        }
              if (gamepad2.right_bumper) {
                  robot.arm.adjustmentPosition -= extensionAddition;
//
              }

              if (gamepad2.left_bumper) {
                  robot.arm.adjustmentPosition += extensionAddition;
              }
//              if (stateMap.get(DRIVE_MODE).equalsIgnoreCase(MANUAL_DRIVE_MODE)) {
//                  if (gamepad1.dpad_down) {
//                      stateMap.put(DRIVE_MODE, AUTO_DRIVE_MODE);
//                      stateMap.put(DRIVE_MODE, AUTO_DRIVE_MODE);
//                      drive.setPoseEstimate(new Pose2d(0, 0, 0));
//                      Pose2d currentPosition = drive.getPoseEstimate();
//                      Pose2d targetPosition = new Pose2d(currentPosition.getX() - 40, currentPosition.getY(), currentPosition.getHeading());
//                      TrajectorySequence reverseTrajectory = drive.highSpeedTrajectoryBuilder(drive.getPoseEstimate())
//                              .lineToLinearHeading(targetPosition)
//                              .UNSTABLE_addTemporalMarkerOffset(-1, () -> toggleMap.put(GAMEPAD_1_A_STATE, false))
//                              .UNSTABLE_addTemporalMarkerOffset(0, () -> stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE))
//                              .build();
//                      drive.followTrajectorySequenceAsync(reverseTrajectory);
//
//                  } else if (gamepad1.dpad_up) {
//                      stateMap.put(DRIVE_MODE, AUTO_DRIVE_MODE);
//                      toggleMap.put(GAMEPAD_1_A_STATE, true);
//                      Pose2d currentPosition = drive.getPoseEstimate();
//                      Pose2d targetPosition = new Pose2d(currentPosition.getX() + 41.5, currentPosition.getY(), currentPosition.getHeading());
//                      TrajectorySequence forwardTrajectory = drive.highSpeedTrajectoryBuilder(drive.getPoseEstimate())
//                              .lineToLinearHeading(targetPosition)
//                              .UNSTABLE_addTemporalMarkerOffset(0, () -> stateMap.put(DRIVE_MODE, MANUAL_DRIVE_MODE))
//                              .build();
//                      drive.followTrajectorySequenceAsync(forwardTrajectory);
//
//                  }
//              }

              if (stateMap.get(DRIVE_MODE).equals(MANUAL_DRIVE_MODE)) {
                  if(!stateMap.get(robot.lift.LIFT_SYSTEM_NAME).equals(robot.lift.LIFT_POLE_GROUND)){
                      drive.setWeightedDrivePower(
                              new Pose2d(
                                      (TRANSLATIONAL_SLOWMODE_SPEED * -gamepad1.left_stick_y),
                                      (TRANSLATIONAL_SLOWMODE_SPEED * -gamepad1.left_stick_x),
                                      (-gamepad1.right_stick_x * TURNING_SLOW_MODE)
                              )
                      );
                  } else if(stateMap.get(robot.flippers.SYSTEM_NAME).equals(robot.flippers.FLIPPERS_DOWN)){
                      drive.setWeightedDrivePower(
                              new Pose2d(
                                      (TRANSLATIONAL_SLOWMODE_SPEED * -gamepad1.left_stick_y),
                                      (TRANSLATIONAL_SLOWMODE_SPEED * -gamepad1.left_stick_x),
                                      (-gamepad1.right_stick_x * FLIPPERS_TURN_SPEED)
                              )
                      );
                  } else {
                      drive.setWeightedDrivePower(
                              new Pose2d(
                                      -gamepad1.left_stick_y,
                                      -gamepad1.left_stick_x,
                                      - REGULAR_TURNING_SPEED * gamepad1.right_stick_x
                              )
                      );
                  }
              }

              drive.update();
              robot.updateSystems();

              telemetry.addData("Cycle Lift up", stateMap.get(constants.CYCLE_LIFT_UP));
              telemetry.addData("Lift selected", stateMap.get(robot.lift.LIFT_SYSTEM_NAME));
              telemetry.addData("Lift subheight adding", robot.lift.liftPickup);
              telemetry.addData("Lift Motor 1 ticks", robot.lift.liftMotor.getCurrentPosition());
              telemetry.addData("Lift Motor 2", robot.lift.liftMotor2.getCurrentPosition());
              telemetry.addData("Toggle map", toggleMap);
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
