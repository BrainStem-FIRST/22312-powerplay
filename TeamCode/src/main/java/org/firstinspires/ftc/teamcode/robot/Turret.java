package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.autoclasses.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret {
    //8192 * 5.23 * 1.8 * 4
    public final double     DEFAULT_TURRET_POWER = 0.1;
    public final double     INITIAL_MOVE_LEFT_TURRET_POWER = 0.1;
    public final double     HARD_STOP_CURRENT_DRAW = 100;

    public final String     SYSTEM_NAME = "TURRET";
    public final String     LEFT_POSITION = "LEFT_STATE";
    public final String     RIGHT_POSITION = "RIGHT_STATE";
    public final String     CENTER_POSITION = "CENTER_STATE";
    public final String     TRANSITION_STATE = "TRANSITION_STATE";



    // Turret position values when the initial position is on the CENTER
    public final int        LEFT_POSITION_VALUE = -256; // 8 -> 0 - (264-8)
    public final int        CENTER_POSITION_VALUE = 0;  // 264 -> 0
    public final int        RIGHT_POSITION_VALUE = 262; // 500 -> 0 + (500-264)


    public final int        ANGLE_TOLERANCE = 5;
    public final int        LIFT_MIN_HEIGHT_TO_MOVE_TURRET = 120;

    public Telemetry telemetry;
    public DcMotorEx turretMotor;

    private PIDController turretPIDController;
    public Extension extension;
    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        //getting turret motor from the hardware map


        turretMotor = (DcMotorEx) hwMap.get("TurretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //PID SETUP
        turretPIDController = new PIDController(0.025, 0.0, 0);
        turretPIDController.setInputBounds(0, 512);
        turretPIDController.setOutputBounds(0, 1);
    }

    public void setState(String desiredState, Lift lift, Extension extension){
        String currentState = getCurrentState();
        if(isLiftTooLow(lift, extension) || desiredState.equalsIgnoreCase(currentState)){
            turretMotor.setPower(0);
        }
        else{
            selectTransition(desiredState, currentState);
        }
    }

    public boolean isLiftTooLow(Lift lift, Extension extension) {
        boolean tooLow = lift.getAvgPosition() < LIFT_MIN_HEIGHT_TO_MOVE_TURRET;
        return tooLow;
    }

    private void selectTransition(String desiredLevel, String currentState){
        switch(desiredLevel){
            case LEFT_POSITION:{
                transitionToPosition(LEFT_POSITION_VALUE);
                break;
            } case CENTER_POSITION:{
                transitionToPosition(CENTER_POSITION_VALUE);
                break;
            } case RIGHT_POSITION:{
                transitionToPosition(RIGHT_POSITION_VALUE);
                break;
            }
        }
    }

    private void transitionToPosition(int ticks){
        //moveTo(ticks);
        moveToPID(ticks);
    }

    public void moveTo (int positionInTicks) {
        // move to desired tick position
        turretMotor.setTargetPosition(positionInTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);
    }
    public void moveToPID (int positionInTicks) {
        // move to desired tick position
        int error = Math.abs(turretMotor.getCurrentPosition() - positionInTicks);
        if (error < 7) {
            turretMotor.setTargetPosition(positionInTicks);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(1.0);
        }
        else if (turretMotor.getCurrentPosition() > positionInTicks) {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setPower(-turretPIDController.updateWithError(error));
        }
        else {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setPower(turretPIDController.updateWithError(error));
        }

        telemetry.addData("Turret Error=", error);
        telemetry.addData("Turret Power=", turretMotor.getPower());
        telemetry.addData("Turret ticks", turretMotor.getCurrentPosition());
    }
    public String getCurrentState() {
        String state = TRANSITION_STATE;
        double currentPosition = getPosition();
        if(inTolerance(currentPosition, LEFT_POSITION_VALUE)){
            state = LEFT_POSITION;
        } else if (inTolerance(currentPosition, CENTER_POSITION_VALUE)) {
            state = CENTER_POSITION;
        } else if (inTolerance(currentPosition, RIGHT_POSITION_VALUE)) {
            state = RIGHT_POSITION;
        }
        return state;
    }

    private boolean inTolerance(double actualTicks, double desiredTicks) {
        return (actualTicks > desiredTicks - ANGLE_TOLERANCE) && (actualTicks < desiredTicks + ANGLE_TOLERANCE);
    }

    public void initializePosition (){
        //set zero position at the stopper to ensure no error with initialization

        while(turretMotor.getCurrent(CurrentUnit.MILLIAMPS) < HARD_STOP_CURRENT_DRAW) {
            turretMotor.setPower(INITIAL_MOVE_LEFT_TURRET_POWER);
        }
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition(){
        return turretMotor.getCurrentPosition();
    }

    public void resetEncoders() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}