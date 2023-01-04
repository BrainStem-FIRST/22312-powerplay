package org.firstinspires.ftc.teamcode.robot.autoclasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;


public class ExtensionA {
    private Telemetry telemetry;

    // Three servos (plus the turret) work together to place cone to desired location
    public ServoImplEx extension;
    public ServoImplEx twoBar;

    static final double MM_TO_INCHES = 0.0393700787;
    static final double MINIMUM_CLEARANCE_DISTANCE = 95.875 * MM_TO_INCHES;
    static final double MAXIMUM_REACH = 11; // inches of extension from position 0 to 1.0

    // Servo Positions
    public final double EXTENSION_POSITION_HOME     = 1.0;   // Fully retracted
    public final double EXTENSION_POSITION_MAX      = 0;     // Fully extended

    public final double EXTENSION_POSITION_LEFT     = 0.5;   // Extend to the pole on the left from center of isle
    public final double EXTENSION_POSITION_RIGHT    = 0.61;  // Extend to the pole on the right from center of isle

    public final double EXTENSION_POSITION_PICKUP   = 0.65;   // Extend to the stack of cones from pickup position
    public final double EXTENSION_POSITION_DEPOSIT  = 0.5;  // Extend to the low pole from pickup position

    public final double EXTENSION_POSITION_SWING_CLEARANCE = 1.0 - (MINIMUM_CLEARANCE_DISTANCE / MAXIMUM_REACH); // 0.63


    public final double TWOBAR_POSITION_HOME    = 0;    // vertical position
    public final double TWOBAR_POSITION_MAX     = 1;    // fully tilted

    // extension statemap values
    public final String SYSTEM_NAME = "EXTENSION"; //statemap key
    public final String DEFAULT_VALUE = "RETRACTED";
    public final String FULL_EXTEND = "EXTENDED_R";
    public final String EXTEND_LEFT = "EXTENDED_L";
    public final String TRANSITION_STATE = "TRANSITION";

    private Map stateMap;

    public double extensionGetPosition(){
        return extension.getPosition();
    }

    public ExtensionA(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        extension = (ServoImplEx) hwMap.servo.get("Extension");
//        twoBar = (ServoImplEx) hwMap.servo.get("Two Bar");

        // Scale the operating range of Servos and set initial position
        extension.setPwmRange(new PwmControl.PwmRange(400,644)); //low cap was 1250 and it was not retracting all the way
        extendHome();

//        twoBar.setPwmRange(new PwmControl.PwmRange(1745,2400));
//        tiltDown();

    }

    /************************* EXTENSION ARM UTILITIES **************************/

    // This method is intended for Teleop mode getting speed value coming from controller (-1..1)
    // Negative speed values will retract the extension arm.

    public void extend(double speed) {
        double currentPosition = extension.getPosition();

        //scale speed value so the extension moves in increments of 10% of the range at max speed
        double targetPosition = Range.clip(currentPosition + speed*0.10, 0, 1);
        extension.setPosition(targetPosition/EXTENSION_POSITION_MAX);

    }

    // Move the extension to the specified position
    // Predefined values can be passed by using class constants (to be defined later)
    // To go as far as it can, pass extension.EXTENSION_MAX_REACH as distance.
    // To go back home, pass 0 as distance.
    public void extendTo(double position) {
        extension.setPosition(position);
    }

/**************************************************************************************
    public final double EXTENSION_MAX_REACH = 10; // TODO: measure actual value in inches and replace this value

    // Moves the extension arm to its clearing length (if it was not already in clear)
    public void getToClear() {
        if (!isInClear()) {
            extendTo(MINIMUM_CLEARANCE_DISTANCE);
        }
    }

    // Returns true if the extension arm's current position is beyond the minimum clearance distance in inches
    public boolean isInClear() {
        double currentPosition = extension.getPosition();

        return (currentPosition * EXTENSION_MAX_REACH) >= MINIMUM_CLEARANCE_DISTANCE;
    }
************************************************************************************/

    // Pulls the extension arm to its starting position (it is NOT in clear)
    public void extendHome() {
        extension.setPosition(EXTENSION_POSITION_HOME);
    }

    public void joyStickExtension(double addedPosition){
        extension.setPosition(extensionGetPosition() + addedPosition);
    }
    // Extends the arm to its maximum reach
    public void extendMax() {
        extension.setPosition(EXTENSION_POSITION_MAX);
    }

    // Extends arm to left position
    public void extendLeft() {
        extension.setPosition(EXTENSION_POSITION_LEFT);
    }

    public void setState(String desiredState){
        selectTransition(desiredState);
    }

    public String getCurrentState() {
        String state = TRANSITION_STATE;
        double currentPosition = extension.getPosition();
        if(currentPosition<0.2){
            state = DEFAULT_VALUE;
        } else if (currentPosition>=0.6) {
            state = FULL_EXTEND;
        } else {
            state = EXTEND_LEFT;
        }
        return state;
    }


    private void selectTransition(String desiredLevel){
        switch(desiredLevel) {
            case DEFAULT_VALUE: {
                extendHome();
                break;
            }
            case FULL_EXTEND: {
                extendMax();
                break;
            }
            case EXTEND_LEFT: {
                extendLeft();
                break;
            }
        }
    }

    public double getExtensionPosition() {
        return extension.getPosition();
    }

    /************************* TWO-BAR UTILITIES **************************/

    // Tilt the two-bar to its max position.
    // No micro adjustments envisioned for the two-bar
    public void tiltUp() {
        twoBar.setPosition(TWOBAR_POSITION_MAX);
    }

    // Tilt the two-bar to its starting position (vertical).
    // No micro adjustments envisioned for the two-bar
    public void tiltDown() {
        twoBar.setPosition(TWOBAR_POSITION_HOME);
    }

    public boolean isExtensionOut(){
        return extensionGetPosition() > 0;
    }

}
