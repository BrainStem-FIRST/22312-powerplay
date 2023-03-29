package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Flippers {
    private Telemetry telemetry;
    //left and right flipper servo created
    public ServoImplEx leftFlipper;
    public ServoImplEx rightFlipper;

    //values of positions
    public final double leftFlipperDown = 1.0;
    public final double leftFlipperUp = 0;
    public final double rightFlipperUp = 1.0;
    public final double rightFlipperDown = 0;

    //Statemap strings and states
    public final String SYSTEM_NAME = "FLIPPERS";
    public final String LEFT_FLIPPER_DOWN = "LEFT_FLIPPER_DOWN";
    public final String RIGHT_FLIPPER_DOWN = "RIGHT_FLIPPER_DOWN";
    public final String LEFT_FLIPPER_UP = "LEFT_FLIPPER_UP";
    public final String RIGHT_FLIPPER_UP = "RIGHT_FLIPPER_UP";
    public final String FLIPPERS_DOWN = "FLIPPERS_DOWN";
    public final String FLIPPERS_UP = "FLIPPERS UP";
    private Map stateMap;

    public Flippers(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        rightFlipper = (ServoImplEx)hwMap.get("RightConeFlipper");
        leftFlipper = (ServoImplEx) hwMap.get("LeftConeFlipper");
        rightFlipper.setPwmRange(new PwmControl.PwmRange(1026,2500));//2500 up and 1000 down
        leftFlipper.setPwmRange(new PwmControl.PwmRange(536,2000));//490 up and 2000 down
        bothFlippersUp();

    }

    public void setState(String desiredState){
        selectTransition(desiredState);
    }

    private void selectTransition(String desiredState){
        switch(desiredState){
            case FLIPPERS_DOWN:{
                bothFlippersDown();
                break;
            }
            case FLIPPERS_UP:{
                bothFlippersUp();
                break;
            }
        }
    }
    public void bothFlippersUp(){
        leftFlipper.setPosition(leftFlipperUp);
        rightFlipper.setPosition(rightFlipperUp);
    }
    public void bothFlippersDown(){
        leftFlipper.setPosition(leftFlipperDown);
        leftFlipper.setPosition(rightFlipperDown);
    }
}
