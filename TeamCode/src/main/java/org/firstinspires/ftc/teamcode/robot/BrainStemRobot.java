package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Thread.sleep;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.java_websocket.extensions.IExtension;

import java.util.HashMap;
import java.util.Map;
import java.util.Timer;

public class BrainStemRobot {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor.RunMode currentDrivetrainMode;

    private Telemetry telemetry;
    private OpMode opMode;

    // declare robot components
    public Turret turret;
    public Lift lift;
    public Extension arm;
    public SampleMecanumDrive drive;
    public Grabber grabber;
    private Map stateMap;
    Constants constants = new Constants();
    private ElapsedTime coneCycleTimer = new ElapsedTime();

    public BrainStemRobot(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.opMode = opMode;

        // instantiate components turret, lift, arm, grabber
        turret = new Turret(hwMap, telemetry);
        lift = new Lift(hwMap, telemetry, stateMap);
        arm = new Extension(hwMap, telemetry, stateMap);
        drive = new SampleMecanumDrive(hwMap);
        grabber = new Grabber(hwMap, telemetry, stateMap);

        // Set run mode (due to lack of a separate initialization function)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    // Not used -> DELETE
    public void initializeRobotPosition() {
        lift.initializePosition();
        lift.moveToMinHeight();  // Raise lift to clear side panels. This does not clear the arm holding cone.
        // Extend the arm so it clears corner of the robot when swinging
        turret.initializePosition();
        lift.raiseHeightTo(0);
    }

    public void updateSystems() {
        //telemetry.addData("robotStateMap" , stateMap);
        stateMap.put(constants.SYSTEM_TIME, System.currentTimeMillis());

//        telemetry.addData("CONE_CYCLE:", stateMap.get(constants.CONE_CYCLE));
//        telemetry.addData("CONE_CYCLE_START_TIME:", stateMap.get(constants.CONE_CYCLE_START_TIME));
//        telemetry.addData("CYCLE_LIFT_DOWN:", stateMap.get(constants.CYCLE_LIFT_DOWN));
//        telemetry.addData("CYCLE_GRABBER:", stateMap.get(constants.CYCLE_GRABBER));
//        telemetry.addData("CYCLE_LIFT_UP:", stateMap.get(constants.CYCLE_LIFT_UP));


        if (((String) stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)) {
            ConeCycleVersion2();
        } else {
            lift.setState();
            turret.setState((String) stateMap.get(turret.SYSTEM_NAME), lift, arm);
            arm.setState((String) stateMap.get(arm.SYSTEM_NAME));
            grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME), lift);

        }
    }

//    public void coneCycle() {
//        if (stateMap.get(constants.CONE_CYCLE_START_TIME) == null) {
//            stateMap.put(constants.CONE_CYCLE_START_TIME, System.currentTimeMillis());
//        }
//
//        if(startLiftDown()) {
//            stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
//            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_IN_PROGRESS);
//        } else if(startGrabberAction()){
//            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_IN_PROGRESS);
//        } else if(startLiftUp()){
//            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_IN_PROGRESS);
//            stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
//        } else if(isConeCycleComplete()){
//            resetConeCycle();
//        }
//
//        setConeCycleSystems();


    public void ConeCycleVersion2() {
        coneCycleTimer.reset();
        if (startGrabberAction()) {
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_IN_PROGRESS);
        } else if (startLiftUp()) {
            telemetry.addData("Cycle Lift up", stateMap.get(constants.CYCLE_LIFT_UP));
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_IN_PROGRESS);
            telemetry.addData("Cycle Lift up", stateMap.get(constants.CYCLE_LIFT_UP));
        } else if (isConeCycleComplete()) {
            resetConeCycle();
        }

        setConeCycleSystems();
    }


    public void resetConeCycle() {
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        coneCycleTimer.reset();
    }
    public void setConeCycleSystems() {
        lift.setState();
        grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME), lift);
        arm.setState((String) stateMap.get(arm.SYSTEM_NAME));
    }

    public long coneCycleStartTime() {
        return (long) stateMap.get(constants.CONE_CYCLE_START_TIME);
    }

    public boolean coneCycleStepTimeExpired(long expirationTime) {
        return (expirationTime + coneCycleStartTime()) < System.currentTimeMillis();
    }

    public boolean startLiftUp() {
        if(stateMap.get(constants.CYCLE_GRABBER).equals(constants.STATE_COMPLETE) && stateMap.get(constants.CYCLE_LIFT_UP).equals(constants.STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }

    private boolean startLiftDown() {
        return (coneCycleStepTimeExpired(constants.GRABBER_CYCLE_TIME) &&
                ((String) stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) &&
                ((String)(stateMap.get(constants.CYCLE_LIFT_DOWN))).equalsIgnoreCase(constants.STATE_NOT_STARTED));
    }

    public boolean startGrabberAction() {
        if(stateMap.get(constants.CONE_CYCLE).equals(constants.STATE_IN_PROGRESS) && stateMap.get(constants.CYCLE_GRABBER).equals(constants.STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }

    public boolean isConeCycleComplete() {
        return (((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_COMPLETE));
    }
    public void updateConeStackState(int cone_count){
        if(cone_count == 5) {
            stateMap.put(constants.DRIVER_2_SELECTED_LIFT, lift.LIFT_CONE_5_STATE);
        } else if(cone_count == 4){
            stateMap.put(constants.DRIVER_2_SELECTED_LIFT, lift.LIFT_CONE_4_STATE);
        } else if(cone_count == 3) {
            stateMap.put(constants.DRIVER_2_SELECTED_LIFT, lift.LIFT_CONE_3_STATE);
        } else if(cone_count == 2){
            stateMap.put(constants.DRIVER_2_SELECTED_LIFT, lift.LIFT_CONE_2_STATE);
        }
    }

    public boolean inConeStack(){
        if(stateMap.get(constants.DRIVER_2_SELECTED_LIFT).equals(lift.LIFT_CONE_5_STATE)){
            return true;
        }
        if(stateMap.get(constants.DRIVER_2_SELECTED_LIFT).equals(lift.LIFT_CONE_4_STATE)){
            return true;
        }
        if(stateMap.get(constants.DRIVER_2_SELECTED_LIFT).equals(lift.LIFT_CONE_3_STATE)){
            return true;
        }
        if(stateMap.get(constants.DRIVER_2_SELECTED_LIFT).equals(lift.LIFT_CONE_2_STATE)){
            return true;
        }
        return false;
    }
}
