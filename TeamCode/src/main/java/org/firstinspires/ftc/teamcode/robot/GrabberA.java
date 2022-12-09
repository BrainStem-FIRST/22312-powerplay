package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Map;



public class GrabberA {
    private Telemetry telemetry;
    private ServoImplEx grabber;


    public final String SYSTEM_NAME = "GRABBER";
    public final String OPEN_STATE = "OPEN";
    public final String CLOSED_STATE = "CLOSED";
    ConstantsA constants = new ConstantsA();

    public final double OPEN_VALUE = 0.21;
    public final double CLOSED_VALUE = 0.12;

    private Map stateMap;

    public GrabberA(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;


        grabber = (ServoImplEx) hwMap.servo.get("Grabber");

        grabber.setPwmRange(new PwmControl.PwmRange(100,2522));
        //grabberOpen();
    }


    public void setState(String position, LiftA lift) {
        if(((String)stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)){
            if (shouldGrab(lift)) {
                grabber.setPosition(CLOSED_VALUE);
            } else {
                grabber.setPosition(OPEN_VALUE);
            }

            if (stateMap.get(constants.GRABBER_START_TIME) == null) {
                stateMap.put(constants.GRABBER_START_TIME, System.currentTimeMillis());
            } else {
                long grabberStartTime = (long) stateMap.get(constants.GRABBER_START_TIME);
                long grabberEndTime = grabberStartTime + constants.GRABBER_CYCLE_TIME;
                if(System.currentTimeMillis() > grabberEndTime) {
                    stateMap.put(constants.GRABBER_START_TIME, null);
                    stateMap.put(constants.CYCLE_GRABBER, constants.STATE_COMPLETE);
                }
            }

        } else if (((String)stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_NOT_STARTED) && shouldGrab(lift)) {
            grabber.setPosition(OPEN_VALUE);
        }
    }
    public boolean shouldGrab(LiftA lift) {
        return lift.getPosition() < lift.LIFT_POSITION_MIDPOLE &&
                ((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS);
    }

    /************************* GRABBER UTILITIES **************************/

    // Opens the claw
    public void grabberOpen() {
        grabber.setPosition(OPEN_VALUE);
    }

    public void grabberClose() {
        grabber.setPosition(CLOSED_VALUE);
    }

    // Returns current position of the grabber. 0 is wide open (dropped cone)
    public double grabberPosition() {
        return grabber.getPosition();
    }
}