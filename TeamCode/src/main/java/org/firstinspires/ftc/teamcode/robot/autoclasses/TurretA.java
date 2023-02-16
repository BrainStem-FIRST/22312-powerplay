package org.firstinspires.ftc.teamcode.robot.autoclasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TurretA {
    //8192 * 5.23 * 1.8 * 4
    public final double     DEFAULT_TURRET_POWER = 0.1;
    public final double     INITIAL_MOVE_LEFT_TURRET_POWER = 0.1;
    public final double     HARD_STOP_CURRENT_DRAW = 100;

    public final String     SYSTEM_NAME = "TURRET";
    public final String     LEFT_POSITION = "LEFT_STATE";
    public final String     RIGHT_POSITION = "RIGHT_STATE";
    public final String     CENTER_POSITION = "CENTER_STATE";
    public final String     TRANSITION_STATE = "TRANSITION_STATE";

    // Needed for Autonomous
    public final String     PICKUP_POSITION = "PICKUP_STATE";
    public final String     DEPOSIT_POSITION = "DEPOSIT_STATE";

    private PIDController turretPIDController;

    // Turret position values when the initial position is on the CENTER
    public final int        LEFT_POSITION_VALUE = -256; // 8 -> 0 - (264-8)
    public final int        CENTER_POSITION_VALUE = 0;  // 264 -> 0
    public final int        RIGHT_POSITION_VALUE = 256; // 500 -> 0 + (500-264)

    // Needed for Autonomous
    public int              turret_PICKUP_POSITION_VALUE = 245;  // These initial values are overwritten by Auto
    public int              turret_PRELOAD_POSITION_VALUE = 152;
    public int              turret_DEPOSIT_POSITION_VALUE = -107;

    public final int        ANGLE_TOLERANCE = 5;
    public final int        LIFT_MIN_HEIGHT_TO_MOVE_TURRET = 60;

    public Telemetry telemetry;
    public DcMotorEx turretMotor;

    private double targetTurretPower = 0.2; // default turret power unless it was overridden

    public TurretA(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        //getting turret motor from the hardware map
        turretMotor = (DcMotorEx) hwMap.get("TurretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // PID controller setup
        turretPIDController = new PIDController(0.07, 0.0, 0);
        turretPIDController.setInputBounds(0, 512);
        turretPIDController.setOutputBounds(0, 1);
    }


    public void transitionToPosition(){
        moveToPID(currentTargetPosition);
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
    }

    // Global variable that holds the turret position that is determined by the Auto classes
    // based on the quadrant and preload/deposit/pickup scenarios.
    // transitionToPosition() passes the value of this variable to moveToPID()
    public int currentTargetPosition = 0;


    public int getPosition(){
        return turretMotor.getCurrentPosition();
    }

    public void resetEncoders() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Pickup and Deposit positions are overridden by the auto classes to fit their own needs
    public void goToDepositPosition() {
        currentTargetPosition = turret_DEPOSIT_POSITION_VALUE;
    }
    public void gotoPreloadPosition(){
        currentTargetPosition = turret_PRELOAD_POSITION_VALUE;
    }
    public void goToPickupPosition() {
        currentTargetPosition = turret_PICKUP_POSITION_VALUE;
    }

    public void setTurretPower(double desiredPower) {
        targetTurretPower = desiredPower;
    }


    ///////////////////////////////////////////////////////////////////////
    //                                                                   //
    //         NOT USED BY AUTO - KEEP FOR FUTURE DEVELOPMENT            //
    //                                                                   //
    ///////////////////////////////////////////////////////////////////////

    public void initializePosition (){
        //set zero position at the stopper to ensure no error with initialization

        while(turretMotor.getCurrent(CurrentUnit.MILLIAMPS) < HARD_STOP_CURRENT_DRAW) {
            turretMotor.setPower(INITIAL_MOVE_LEFT_TURRET_POWER);
        }
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        } else if (inTolerance(currentPosition, turret_PICKUP_POSITION_VALUE)) {
            state = PICKUP_POSITION;
        } else if (inTolerance(currentPosition, turret_DEPOSIT_POSITION_VALUE)) {
            state = DEPOSIT_POSITION;
        }
        return state;
    }

    private boolean inTolerance(double actualTicks, double desiredTicks) {
        return (actualTicks > desiredTicks - ANGLE_TOLERANCE) && (actualTicks < desiredTicks + ANGLE_TOLERANCE);
    }

    /////////////////////////////////////////////////////
    //  NOT USED BY AUTO. Instead, use moveToPID
    /////////////////////////////////////////////////////
    public void moveTo (int positionInTicks) {
        // move to desired tick position
        currentTargetPosition = positionInTicks;
        turretMotor.setTargetPosition(positionInTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Experiment: Slow down the turret to see if it stops on target
        turretMotor.setPower(targetTurretPower);
    }

    public void setState(String desiredState, LiftA lift){
        String currentState = getCurrentState();
        if(isLiftTooLow(lift) || desiredState.equalsIgnoreCase(currentState)){
            turretMotor.setPower(0);
        }
        else{
//            telemetry.addData("Desired:", desiredState);
//            telemetry.addData("Current", currentState);
            selectTransition(desiredState, currentState);
        }
    }

    public boolean isLiftTooLow(LiftA lift) {
        boolean tooLow = lift.getPosition() < LIFT_MIN_HEIGHT_TO_MOVE_TURRET;
        return tooLow;
    }

    ////////////////////////////////////////////////////////////////////////
    // Auto does not use selectTransition at this time.
    // Commented out transitionToPosition
    ////////////////////////////////////////////////////////////////////////
    private void selectTransition(String desiredLevel, String currentState){
        switch(desiredLevel){
            case LEFT_POSITION: {
                telemetry.addData("Left at: ", LEFT_POSITION_VALUE);
//                transitionToPosition(LEFT_POSITION_VALUE);
                break;
            } case CENTER_POSITION:{
                telemetry.addData("Center at: ", CENTER_POSITION_VALUE);
//                transitionToPosition(CENTER_POSITION_VALUE);
                break;
            } case RIGHT_POSITION:{
                telemetry.addData("Right at: ", RIGHT_POSITION_VALUE);
//                transitionToPosition(RIGHT_POSITION_VALUE);
                break;
            } case PICKUP_POSITION:{
                telemetry.addData("Pickup at: ", turret_PICKUP_POSITION_VALUE);
//                transitionToPosition(turret_PICKUP_POSITION_VALUE);
                break;
            } case DEPOSIT_POSITION:{
                telemetry.addData("Deposit at: ", turret_DEPOSIT_POSITION_VALUE);
//                transitionToPosition(turret_DEPOSIT_POSITION_VALUE);
                break;
            }
        }
    }


}
