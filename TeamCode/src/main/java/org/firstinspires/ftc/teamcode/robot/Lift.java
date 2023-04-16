package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.autoclasses.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;


public class Lift {
    private Telemetry telemetry;
    public DcMotorEx liftMotor;
    public DcMotorEx liftMotor2;
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
    public int LIFT_POSITION_GROUND = 0;
    public final int GROUND_ORIGINAL_POSITION = 0;
    public int LIFT_POSITION_LOWPOLE = 420;
    public int LIFT_POSITION_MIDPOLE = 640;   //685;
    public int LIFT_POSITION_HIGHPOLE = 850;
    public final int LIFT_POSITION_MOVING = 100;

    public final int LIFT_POSITION_CONE_5 = 200;
    public final int LIFT_POSITION_CONE_4 = 155;
    public final int LIFT_POSITION_CONE_3 = 100;
    public final int LIFT_POSITION_CONE_2 = 60;

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
    public final String LIFT_CONE_5_STATE = "LIFT_CONE_5_STATE";
    public final String LIFT_CONE_4_STATE = "LIFT_CONE_4_STATE";
    public final String LIFT_CONE_3_STATE = "LIFT_CONE_3_STATE";
    public final String LIFT_CONE_2_STATE = "LIFT_CONE_2_STATE";

    // Used in Auto to determine the lift's position high enough to unstack the cones during pickup
    public final String LIFT_POSITION_CLEAR = "LIFT_CLEAR_HEIGHT";
    // This is the encoder tick count for the lift that raises the cone's base just below the rim of the field wall.
    // Raising the cone any further during auto pickup risks hitting the cone's base to the lip of the wall.
    public final int LIFT_CLEAR_HEIGHT = 285;   // Encoder position was determined empirically

    public final String TRANSITION_STATE = "TRANSITION";
    public final int DELIVERY_ADJUSTMENT = -3;
    public final int HEIGHT_TOLERANCE = 5;
    public final int CYCLE_TOLERANCE = 5;
    public final String LIFT_CURRENT_STATE = "LIFT CURRENT STATE";

    public static double currentLiftHeight;
    private Map stateMap;
    private int adjustmentHeight;
    public int liftPickup;
    private PIDController liftController;


    public Lift(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        liftController = new PIDController(0.0125,0.,0);
        liftMotor = hwMap.get(DcMotorEx.class, "Lift");
        liftMotor2 = hwMap.get(DcMotorEx.class, "LiftMotor2");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        liftController.setInputBounds(0, 850);
        liftController.setOutputBounds(-0.2, 1.0);

    }

    public boolean isCollectionHeight() {
        return getAvgPosition() < (LIFT_POSITION_GROUND + CYCLE_TOLERANCE);
    }

    public int getAvgPosition() {
        telemetry.addData("Lift motor average", (liftMotor.getCurrentPosition() + liftMotor2.getCurrentPosition()) / 2 );
        return (liftMotor.getCurrentPosition() + liftMotor2.getCurrentPosition()) / 2;
    }

    public void setState() {
        String subheight = (String) stateMap.get(LIFT_SUBHEIGHT);
        telemetry.addData("Subheight in lift class", subheight);
        String currentState = getCurrentState(subheight);

        telemetry.addData("Current state", currentState);

        String level = (String) stateMap.get(LIFT_SYSTEM_NAME);
        telemetry.addData("Desired Level", level);
        stateMap.put(LIFT_CURRENT_STATE, currentState);
        telemetry.addData("stateMap call to lift current state", stateMap.get(LIFT_CURRENT_STATE));
        telemetry.addData("shouldLifMove", shouldLiftMove(level,currentState));

        setMotorPidPower(selectTransition(level, subheight, currentState));
//        if(shouldLiftMove(level, currentState)) {
//            selectTransition(level, subheight, currentState);
//            telemetry.addData("Lift Motor 1 Power", liftMotor.getPower());
//            telemetry.addData("Lift Motor 2 Power", liftMotor2.getPower());
//            telemetry.addData("Lift Motor 1 is busy", liftMotor.isBusy());
//            telemetry.addData("Lift Motor 2 is busy", liftMotor2.isBusy());
//            telemetry.addLine("In lift should move true");
//
//        } else {
//            if(stateMap.get(LIFT_SYSTEM_NAME).equals(LIFT_POLE_GROUND)){
//                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftMotor.setPower(0);
//                liftMotor2.setPower(0);
//            } else {
//                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftMotor.setPower(0.15);
//                liftMotor2.setPower(0.15);
//            }
//            telemetry.addData("Lift Motor 1 Power", liftMotor.getPower());
//            telemetry.addData("Lift Motor 2 Power", liftMotor2.getPower());
//            telemetry.addData("Lift Motor 1 is busy", liftMotor.isBusy());
//            telemetry.addData("Lift Motor 2 is busy", liftMotor2.isBusy());
//            telemetry.addLine("In lift should move else");
//        }
    }

    private void setMotorPidPower(int ticks) {
        if (ticks != liftController.getTarget()) {
            liftController.reset();
            liftController.setTarget(ticks);
        }

        double power = liftController.update(getAvgPosition());
        setRawPower(-power);
        telemetry.addData("raw power from pid:", power);
    }

    public void setAdjustmentHeight(double driverInput) {
        adjustmentHeight = (int) (400 * driverInput);
        telemetry.addData("adjustment height", adjustmentHeight);
        telemetry.addData("high pole ticks", LIFT_POSITION_HIGHPOLE);
    }

    private boolean shouldLiftMove(String level, String currentState) {
        return (!level.equalsIgnoreCase(currentState));
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
        if(isCycleInProgress(constants.CYCLE_LIFT_UP) && inHeightTolerance(getAvgPosition(), position) ){
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_COMPLETE);
        } else if(isCycleInProgress(constants.CYCLE_LIFT_UP) && !stateMap.get(LIFT_SYSTEM_NAME).equals(LIFT_UP_MOVING_STATE)){
            stateMap.put(LIFT_SYSTEM_NAME, LIFT_UP_MOVING_STATE);
        }
    }

    public void setRawPower(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setPower(power);
        liftMotor.setPower(power);
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
    public void raiseHeightPID(int heightInTicks){
        int position = getAvgPosition();
//        if (position < (heightInTicks - 200)) {
//            setRawPower(1.0);
//        } else if (position > (heightInTicks + 150)) {
//            setRawPower(-0.7);
//        } else if (heightInTicks == 0 && (position < heightInTicks + 15)) {
//            setRawPower(0.0);
//        }else if (position <= heightInTicks - HEIGHT_TOLERANCE || position >= heightInTicks + HEIGHT_TOLERANCE) { // THIS IS THE RANGE THAT IT SITS IN WHEN ITS SET TO A PLACE TO GO
////            if (stateMap.get(LIFT_SYSTEM_NAME) == LIFT_POLE_GROUND &&  heightInTicks > 0 && position < 30) {
////                runAllMotorsToPosition(heightInTicks, 1);
////            } else {
////                runAllMotorsToPosition(heightInTicks, 0.6);
////            }
//            if(stateMap.get(LIFT_SYSTEM_NAME).equals(LIFT_POLE_GROUND)){
//                if(position >= 300){
//                    runAllMotorsToPosition(heightInTicks, 1.0);
//                } else if(position < 300 && position >= 150) {
//                    runAllMotorsToPosition(heightInTicks, 0.5);
//                }
//            } else {
//                if(position >= 300 && position <= heightInTicks - 20){
//                    runAllMotorsToPosition(heightInTicks, 1.0);
//                    Log.d("BrainSTEM 22312 RC ", "raiseHeightPID: ");
//                } else if(position > heightInTicks - 20 && position <= heightInTicks - 10){
//                    runAllMotorsToPosition(heightInTicks, 0.85);
//                    telemetry.addLine("0.85 power");
//                } else {
//                    runAllMotorsToPosition(heightInTicks, 0.15);
//                }
//            }
//        }  else {
//            setRawPower(0.15);
//            Log.d("BRAINSTEM 22312 RC", "" + liftMotor.getPower());
//        }
        if (position < (heightInTicks - 200)) {
            setRawPower(1.0);
        } else if (position > (heightInTicks + 150)) {
            setRawPower(-0.7);
        } else if (heightInTicks == 0 && (position < heightInTicks + 15)) {
            setRawPower(0.0);
        }else if (position <= heightInTicks - 7 || position >= heightInTicks + 7) { // THIS IS THE RANGE THAT IT SITS IN WHEN ITS SET TO A PLACE TO GO
            if (stateMap.get(LIFT_SYSTEM_NAME) == LIFT_POLE_GROUND &&
                    heightInTicks > 0 &&
                    position < 30) {
                runAllMotorsToPosition(heightInTicks, 1);
            } else if (heightInTicks > 300){
                runAllMotorsToPosition(heightInTicks, 0.5);
            } else {
                runAllMotorsToPosition(heightInTicks, 0.3);
            }
        }  else {
            setRawPower(0.15);
        }
    }

    public void runAllMotorsToPosition(int ticks, double power){
        //raising heights to reach different junctions, so four values
        liftMotor.setTargetPosition(ticks);
        liftMotor2.setTargetPosition(ticks);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setTargetPositionTolerance(2);
//        liftMotor2.setTargetPositionTolerance(2);
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
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
                position = LIFT_POSITION_GROUND + liftPickup;
                break;
            }
        }
        // this function can return position 0 if lift is in transition between heights (as reported by getCurrentState() function)
        telemetry.addData("Lift State Position =", position);
        return position;
    }


    private int selectTransition(String desiredLevel, String subheight, String currentState){
        int targetHeight = 0;
        switch(desiredLevel){
            case LIFT_POLE_LOW:{
                targetHeight = LIFT_POSITION_LOWPOLE - adjustmentHeight;
                break;
            }
            case LIFT_POLE_MEDIUM:{
                targetHeight = LIFT_POSITION_MIDPOLE - adjustmentHeight;
                break;
            }
            case LIFT_POLE_HIGH:{
                targetHeight = LIFT_POSITION_HIGHPOLE - adjustmentHeight;
                break;
            }
            case LIFT_POLE_GROUND:{
                targetHeight = LIFT_POSITION_GROUND + liftPickup;
                break;
            }
        }
        return targetHeight;
    }

    private void transitionToLiftPosition(int ticks){
        telemetry.addData("target heights", ticks);
        raiseHeightPID(ticks);
    }

    public String getCurrentState(String subheight) {
        String state = TRANSITION_STATE;
        double currentPosition = getAvgPosition();
        telemetry.addData("currentPosition", currentPosition);
        telemetry.addData("liftPositionPickup", liftPositionPickup);
//        telemetry.addData("deliveryHeight(subheight)", deliveryHeight(subheight));
        if(inHeightTolerance(currentPosition, LIFT_POSITION_GROUND + liftPickup)){
            state = LIFT_POLE_GROUND;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_LOWPOLE)) {
            state = LIFT_POLE_LOW;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_MIDPOLE)) {
            state = LIFT_POLE_MEDIUM;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_HIGHPOLE)) {
            state = LIFT_POLE_HIGH;
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
//        liftMotor.setTargetPositionTolerance(2);
//        liftMotor2.setTargetPositionTolerance(2);
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
        return (heightPosition >= (targetHeight - HEIGHT_TOLERANCE)) && (heightPosition <= (targetHeight + HEIGHT_TOLERANCE));
    }
    public boolean isLiftUp(){
        return (getAvgPosition() > LIFT_POSITION_GROUND);
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
        liftMotor.setTargetPosition(getAvgPosition() - 20);
        liftMotor2.setTargetPosition(getAvgPosition() - 20);
        liftMotor.setPower(1.0);
        liftMotor2.setPower(1.0);
    }
}