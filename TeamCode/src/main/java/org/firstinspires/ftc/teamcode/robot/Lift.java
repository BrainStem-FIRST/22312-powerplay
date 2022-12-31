package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;


public class Lift {
    private Telemetry telemetry;
    public DcMotor liftMotor;
    public DcMotor liftMotor2;
    static final double MM_TO_INCHES = 0.0393700787;

    static final double COUNTS_PER_MOTOR_REV = 28;     // ticks at the motor shaft
    static final double DRIVE_GEAR_REDUCTION = 2.89;     // 3:1 gear reduction (slowing down) is actual 2.89:1
    static final double PULLEY_WHEEL_DIAMETER_INCHES = 24.25 * MM_TO_INCHES;     // convert mm to inches
    static final double TICK_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double LIFT_UP_SPEED = 0.5;
    static final double LIFT_DOWN_SPEED = 0.5;

    public final int MINIMUM_CLEARANCE_HEIGHT = 43;    // inches to lift to clear side panels

    public final double CONE_BASE = 1.3;    //base of the cone for pickup calculations

    // TODO: Pole heights might need to be recalculated because the lift starting position (encoder values reset) is now the height of single cone at hand
    public final int LIFT_POSITION_RESET = 0;

    public final int LIFT_ADJUSTMENT = -70;

    // Empirical numbers are for holding the cone above the pole prior to coneCycle drop
    public final int LIFT_POSITION_GROUND = 0;
    public final int LIFT_POSITION_LOWPOLE = 450;
    public final int LIFT_POSITION_MIDPOLE = 700;   //685;
    public final int LIFT_POSITION_HIGHPOLE = 960;
    public final int LIFT_POSITION_MOVING = 10;

    // Lift pick up position is only 4 cone bases higher than the starting position,
    // which is reset to 0 ticks at the start of Auto when lift is positioned on top of a single cone
//    public final int LIFT_PICKUP_INIT = (int) ((CONE_BASE * 4) * TICK_PER_INCH);
//    public int liftPositionPickup = LIFT_PICKUP_INIT - LIFT_ADJUSTMENT;

    public int numCyclesCompleted = 0;      //numCyclesCompleted during Auto for pickup calculations
    public int liftPositionPickup = 165 - LIFT_ADJUSTMENT;

    Constants constants = new Constants();


    public final double HARD_STOP_CURRENT_DRAW = 100;

    public final String LIFT_SYSTEM_NAME = "Lift";
    public final String LIFT_PICKUP = "PICKUP";
    public final String LIFT_POLE_GROUND = "GROUND";
    public final String LIFT_POLE_LOW = "POLE_LOW";
    public final String LIFT_POLE_MEDIUM = "POLE_MEDIUM";
    public final String LIFT_POLE_HIGH = "POLE_HIGH";
    public final String APPROACH_HEIGHT = "APPROACH_HEIGHT";
    public final String PLACEMENT_HEIGHT = "PLACEMENT_HEIGHT";
    public final String LIFT_SUBHEIGHT = "SUB_HEIGHT";
    public final String LIFT_CHECK_STATE = "LIFT_CHECK";
    public final String LIFT_UP_MOVING_STATE = "LIFT_UP_MOVING_STATE";

    // Used in Auto to determine the lift's position high enough to unstack the cones during pickup
    public final String LIFT_POSITION_CLEAR = "LIFT_CLEAR_HEIGHT";
    // This is the encoder tick count for the lift that raises the cone's base just below the rim of the field wall.
    // Raising the cone any further during auto pickup risks hitting the cone's base to the lip of the wall.
    public final int LIFT_CLEAR_HEIGHT = 285;   // Encoder position was determined empirically

    public final String TRANSITION_STATE = "TRANSITION";
    public final int DELIVERY_ADJUSTMENT = -3;
    public final int HEIGHT_TOLERANCE = 10;
    public final int CYCLE_TOLERANCE = 5;
    public final String LIFT_CURRENT_STATE = "LIFT CURRENT STATE";

    public static double currentLiftHeight;
    private Map stateMap;
    private int adjustmentHeight;


    public Lift(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        liftMotor = hwMap.dcMotor.get("Lift");
        liftMotor2 = hwMap.dcMotor.get("LiftMotor2");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
    }

    public boolean isCollectionHeight() {
        return getPosition() < (LIFT_POSITION_GROUND + CYCLE_TOLERANCE);
    }

    public int getPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void setState() {
        String subheight = (String) stateMap.get(LIFT_SUBHEIGHT);
        String currentState = getCurrentState(subheight);
        String level = (String) stateMap.get(LIFT_SYSTEM_NAME);

        stateMap.put(LIFT_CURRENT_STATE, currentState);

        updateConeCycleState();

        if (shouldLiftMove(level, currentState) ) {
            selectTransition(level, subheight, currentState);
        } else {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
        }
    }

    public void setAdjustmentHeight(double driverInput) {
        adjustmentHeight = (int) (250 * driverInput);
    }

    private boolean shouldLiftMove(String level, String currentState) {
        return (((String)stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) ||
                !level.equalsIgnoreCase(currentState));
    }

    private void updateConeCycleState() {
        int position = getStateValue();
//        if (isCycleInProgress(constants.CYCLE_LIFT_DOWN) && isSubheightPlacement()) {
//            if (inHeightTolerance(getPosition(), position + LIFT_ADJUSTMENT)) {
//                stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_COMPLETE);
//            }
//        } else if (isCycleInProgress(constants.CYCLE_LIFT_UP) && inHeightTolerance(getPosition(), position)) {
//            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
//        } else if (isCycleInProgress(constants.CYCLE_LIFT_UP) && coneCycleStepTimeExpired(constants.GRABBER_CYCLE_TIME + constants.LIFT_DOWN_CYCLE_TIME + constants.LIFT_UP_CYCLE_TIME)) {
//            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
//        } else if (isCycleInProgress(constants.CYCLE_LIFT_DOWN) && coneCycleStepTimeExpired(constants.GRABBER_CYCLE_TIME + constants.LIFT_DOWN_CYCLE_TIME)) {
//            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_COMPLETE);
//        }
        if( isCycleInProgress(constants.CYCLE_LIFT_UP) && inHeightTolerance(getPosition(), position) ){
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
        } else if(isCycleInProgress(constants.CYCLE_LIFT_UP) && !((String)(stateMap.get(LIFT_SYSTEM_NAME))).equalsIgnoreCase(LIFT_UP_MOVING_STATE)){
            stateMap.put(LIFT_SYSTEM_NAME, LIFT_UP_MOVING_STATE);
        }
    }

    private long coneCycleStartTime() {
        return (long) stateMap.get(constants.CONE_CYCLE_START_TIME);
    }

    private boolean coneCycleStepTimeExpired(long expirationTime) {
        if (stateMap.get(constants.CONE_CYCLE_START_TIME) == null) {
            return false;
        } else {
            return (expirationTime + coneCycleStartTime()) < System.currentTimeMillis();
        }
    }


    private boolean isCycleInProgress(String cycleName) {
        return ((String)stateMap.get(cycleName)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
    }

    private boolean isSubheightPlacement(){
        return ((String)stateMap.get(LIFT_SUBHEIGHT)).equalsIgnoreCase(PLACEMENT_HEIGHT);
    }
    private int getStateValue(){
        int position = 0;
        switch((String)stateMap.get(LIFT_CURRENT_STATE)) {
            case LIFT_POLE_HIGH: {
                position = LIFT_POSITION_HIGHPOLE;
                break;
            }
            case LIFT_POLE_MEDIUM: {
                position = LIFT_POSITION_MIDPOLE;
                break;
            }
            case LIFT_POLE_LOW: {
                position = LIFT_POSITION_LOWPOLE;
                break;
            }
            case LIFT_POLE_GROUND:{
                position = LIFT_POSITION_GROUND;
                break;
            }
            case LIFT_POSITION_CLEAR:{
                position = LIFT_CLEAR_HEIGHT;
                break;
            }
            case LIFT_PICKUP:{
                // accounts for stacked cone height
                position = liftPositionPickup;
                break;
            }
            case LIFT_UP_MOVING_STATE:{
                position = LIFT_POSITION_MOVING;
                break;
            }
        }
        // this function can return position 0 if lift is in transition between heights (as reported by getCurrentState() function)
//        telemetry.addData("Lift State Position =", position);
        return position;
    }


    private void selectTransition(String desiredLevel, String subheight, String currentState){
        switch(desiredLevel){
            case LIFT_PICKUP:{
                transitionToLiftPosition(liftPositionPickup + deliveryHeight(subheight));
                break;
            }
            case LIFT_POLE_LOW:{
                transitionToLiftPosition(LIFT_POSITION_LOWPOLE - adjustmentHeight);
                break;
            }
            case LIFT_POLE_MEDIUM:{
                transitionToLiftPosition(LIFT_POSITION_MIDPOLE - adjustmentHeight);
                break;
            }
            case LIFT_POLE_HIGH:{
                transitionToLiftPosition(LIFT_POSITION_HIGHPOLE - adjustmentHeight);
                break;
            }
            case LIFT_POLE_GROUND:{
                transitionToLiftPosition(LIFT_POSITION_GROUND + deliveryHeight(subheight));
                break;
            }
            case LIFT_POSITION_CLEAR:{
                transitionToLiftPosition(LIFT_CLEAR_HEIGHT);
                break;
            }
            case LIFT_CHECK_STATE:{
                checkLift();
                break;
            }
            case LIFT_UP_MOVING_STATE:{
                transitionToLiftPosition(LIFT_POSITION_MOVING);
                break;
            }
        }

    }
    private void transitionToLiftPosition(int ticks){
        raiseHeightTo(ticks);
    }

    public String getCurrentState(String subheight) {
        String state = TRANSITION_STATE;
        double currentPosition = getPosition();
        telemetry.addData("currentPosition", currentPosition);
        telemetry.addData("liftPositionPickup", liftPositionPickup);
        telemetry.addData("deliveryHeight(subheight)", deliveryHeight(subheight));
        if(inHeightTolerance(currentPosition, LIFT_POSITION_GROUND + deliveryHeight(subheight))){
            state = LIFT_POLE_GROUND;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_LOWPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_LOW;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_MIDPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_MEDIUM;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_HIGHPOLE + deliveryHeight(subheight))) {
            state = LIFT_POLE_HIGH;
        } else if (inHeightTolerance(currentPosition, LIFT_CLEAR_HEIGHT)) {
            state = LIFT_POSITION_CLEAR;
        } else if (inHeightTolerance(currentPosition, liftPositionPickup + deliveryHeight(subheight))) {
            state = LIFT_PICKUP; //accounted for pickup
        } else if(inHeightTolerance(currentPosition, LIFT_POSITION_MOVING)){
            state = LIFT_UP_MOVING_STATE;
        }
        return state;
    }

    public int deliveryHeight(String subheight){
        int height = 0;

        if(subheight.equalsIgnoreCase(PLACEMENT_HEIGHT)){
            height += LIFT_ADJUSTMENT;
        }
        return height;
    }

    public void raiseHeightTo (int heightInTicks) {
        //raising heights to reach different junctions, so four values
        liftMotor.setTargetPosition(heightInTicks);
        liftMotor2.setTargetPosition(heightInTicks);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
        liftMotor2.setPower(1.0);

    }

    // Not used -> DELETE
    public  boolean isClear () {
        //true means turret can turn and lift is raised to minimum clearance; false is the opposite
        double currentLiftHeight = liftMotor.getCurrentPosition() * TICK_PER_INCH;
        if(currentLiftHeight >= MINIMUM_CLEARANCE_HEIGHT){
            return true;
        }
        return false;

    }
    // Not used -> DELETE
    public void moveToMinHeight(){
        if (!isClear()) {
            raiseHeightTo(MINIMUM_CLEARANCE_HEIGHT);
        }
    }

    // Not used -> DELETE
    public void initializePosition( ) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMotor(double power){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    private boolean inHeightTolerance(double heightPosition, double targetHeight) {
        return (heightPosition > targetHeight - HEIGHT_TOLERANCE) && (heightPosition < targetHeight + HEIGHT_TOLERANCE);
    }
    public boolean isLiftUp(){
        return (getPosition() > LIFT_POSITION_GROUND);
    }

    // Used by Auto to reduce lift pickup position each time the number of cones in the stack were removed
    public void updateLiftPickupPosition() {
        switch (numCyclesCompleted){
            case 0: {
                liftPositionPickup = 185-LIFT_ADJUSTMENT; //170
                break;
            }
            case 1: {
                liftPositionPickup = 140-LIFT_ADJUSTMENT; //130
                break;
            }
            case 2: {
                liftPositionPickup = 110-LIFT_ADJUSTMENT; //100
                break;
            }
            case 3: {
                liftPositionPickup = 70-LIFT_ADJUSTMENT;
                break;
            }
            case 4: {
                liftPositionPickup = 40-LIFT_ADJUSTMENT;
                break;
            }
        }
    }

    public void resetEncoders() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
//    public void checkCone(int encoderTicks){
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setTargetPosition(getPosition() + encoderTicks);
//        liftMotor2.setTargetPosition(getPosition() + encoderTicks);
//        liftMotor.setPower(0.5);
//        liftMotor2.setPower(0.5);
//
//    }
    public void checkLift(){
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(getPosition() - 20);
        liftMotor2.setTargetPosition(getPosition() - 20);
        liftMotor.setPower(1.0);
        liftMotor2.setPower(1.0);
    }
}