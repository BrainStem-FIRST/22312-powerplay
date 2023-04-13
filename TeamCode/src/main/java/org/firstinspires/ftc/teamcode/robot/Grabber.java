package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Map;


public class Grabber {
    private Telemetry telemetry;
    private ServoImplEx grabber;


    public final String SYSTEM_NAME = "GRABBER";
    public final String OPEN_STATE = "OPEN";
    public final String CLOSED_STATE = "CLOSED";
    public final String FULLY_OPEN_STATE = " FULLY OPEN STATE";
    public final String CLOSED_COMPLETELY = "COMPLETELY CLOSED STATE";
    Constants constants = new Constants();

    public final double FULLY_OPEN_VALUE = 0.00;
    private final double CONE_OPEN_VALUE = 0.4; //0.55
    public final double CLOSED_VALUE = 1;
    public final double COMPLETELY_CLOSED_VALUE = 1.0;
    public ElapsedTime grabberCycleTime;


    private Map stateMap;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.grabberCycleTime = new ElapsedTime();

        grabber = (ServoImplEx) hwMap.servo.get("Grabber");

        grabber.setPwmRange(new PwmControl.PwmRange(1200, 2100));
        //grabberOpen();
    }


    public void setState(String position, Lift lift) {
        if (((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)) {
            if (shouldGrab(lift)) {
                grabber.setPosition(CLOSED_VALUE);
            } else {
                grabber.setPosition(CONE_OPEN_VALUE);
            }

//            if (stateMap.get(constants.GRABBER_START_TIME) == null) {
//                stateMap.put(constants.GRABBER_START_TIME, System.currentTimeMillis());
//            } else {
//                long grabberStartTime = (long) stateMap.get(constants.GRABBER_START_TIME);
//                long grabberEndTime = grabberStartTime + constants.GRABBER_CYCLE_TIME;
            if (grabberCycleTime.milliseconds() > constants.GRABBER_CYCLE_TIME) {
                stateMap.put(constants.CYCLE_GRABBER, constants.STATE_COMPLETE);
            }
            telemetry.addData("Grabber time", grabberCycleTime.milliseconds());
        } else if (((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_NOT_STARTED) && shouldGrab(lift)) {
            grabber.setPosition(CONE_OPEN_VALUE);
        } else if (((String) stateMap.get(SYSTEM_NAME)).equalsIgnoreCase(FULLY_OPEN_STATE)) {
            if (shouldGrabberFullyOpen(lift)) {
                grabber.setPosition(FULLY_OPEN_VALUE);
            } else {
                grabber.setPosition(CONE_OPEN_VALUE);
            }
        } else if (((String) stateMap.get(SYSTEM_NAME)).equalsIgnoreCase(OPEN_STATE)) {
            grabber.setPosition(CONE_OPEN_VALUE);
        } else if (((String) stateMap.get(SYSTEM_NAME)).equalsIgnoreCase(CLOSED_STATE)) {
            grabber.setPosition(CLOSED_VALUE);
    } else if(((String) stateMap.get(SYSTEM_NAME)).equalsIgnoreCase(CLOSED_COMPLETELY)){
            if(shouldGrabberFullyClose(lift)) {
                grabber.setPosition(COMPLETELY_CLOSED_VALUE);
            } else {
                grabber.setPosition(CONE_OPEN_VALUE);
            }
        }
    }

    public boolean shouldGrab(Lift lift) {
        return lift.getAvgPosition() < lift.LIFT_POSITION_MIDPOLE &&
                ((String) stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
    }

    public boolean shouldGrabberFullyClose(Lift lift){
        return lift.getAvgPosition() > 100;
    }

    /************************* GRABBER UTILITIES **************************/

    // Opens the claw
    public void grabberOpen() {
        grabber.setPosition(CONE_OPEN_VALUE);
    }

    public void grabberClose() {
        grabber.setPosition(CLOSED_VALUE);
    }

    public static boolean shouldGrabberFullyOpen(Lift lift) {
        if (lift.getAvgPosition() > 300) {
            return true;
        }
        return false;
    }

    // Returns current position of the grabber. 0 is wide open (dropped cone)
    public double grabberPosition() {
        return grabber.getPosition();
    }
}