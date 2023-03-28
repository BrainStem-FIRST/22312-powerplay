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
    public final double rightFlipperUp = 0.0;
    public final double rightFlipperDown = 1.0;

    //Statemap strings and states
    public final String SYSTEM_NAME = "FLIPPERS";
    public final String LEFT_FLIPPER_DOWN = "LEFT_FLIPPER_DOWN";
    public final String RIGHT_FLIPPER_DOWN = "RIGHT_FLIPPER_DOWN";
    public final String LEFT_FLIPPER_UP = "LIFT_FLIPPER_UP";
    public final String RIGHT_FLIPPER_UP = "RIGHT_FLIPPER_UP";
    private Map stateMap;

    public Flippers(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        rightFlipper = (ServoImplEx)hwMap.get("RightConeFlipper");
        leftFlipper = (ServoImplEx) hwMap.get("LeftConeFlipper");
        rightFlipper.setPwmRange(new PwmControl.PwmRange(1000,2500));//2500 up and 1000 down
        leftFlipper.setPwmRange(new PwmControl.PwmRange(490,2000));//490 up and 2000 down
        rightFlipperUp();
        leftFlipperUp();
    }

    public void setState(String desiredState){
        selectTransition(desiredState);
    }

    private void selectTransition(String desiredState){
        switch(desiredState){
            case LEFT_FLIPPER_DOWN:{
                leftFlipperDown();
                break;
            }
            case RIGHT_FLIPPER_DOWN:{
                rightFlipperDown();
                break;
            }
            case LEFT_FLIPPER_UP:{
                leftFlipperUp();
                break;
            }
            case RIGHT_FLIPPER_UP:{
                rightFlipperUp();
                break;
            }
        }
    }
    private void leftFlipperDown(){
        leftFlipper.setPosition(leftFlipperDown);
    }
    private void rightFlipperDown(){
        leftFlipper.setPosition(leftFlipperUp);
    }
    private void leftFlipperUp(){
        leftFlipper.setPosition(rightFlipperUp);
    }
    private void rightFlipperUp(){
        rightFlipper.setPosition(rightFlipperDown);
    }
}
