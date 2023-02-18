package org.firstinspires.ftc.teamcode.robot.autoclasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Map;

public class BrainStemRobotA {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor.RunMode currentDrivetrainMode;

    private Telemetry telemetry;
    private OpMode opMode;

    // declare robot components
    public TurretA turret;
    public LiftA lift;
    public ExtensionA arm;
    public SampleMecanumDrive drive;
    public GrabberA grabber;
    private Map stateMap;
    ConstantsA constants = new ConstantsA();

    public BrainStemRobotA(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.opMode = opMode;

        // instantiate components turret, lift, arm, grabber
        turret  = new TurretA(hwMap, telemetry);
        lift    = new LiftA(hwMap, telemetry, stateMap);
        arm     = new ExtensionA(hwMap, telemetry, stateMap);
        drive   = new SampleMecanumDrive(hwMap);
        grabber   = new GrabberA(hwMap, telemetry, stateMap);

        // Set run mode (due to lack of a separate initialization function)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    public void updateSystems() {
        //telemetry.addData("robotStateMap" , stateMap);
        stateMap.put(constants.SYSTEM_TIME, System.currentTimeMillis());

        telemetry.addData("CONE_CYCLE:", stateMap.get(constants.CONE_CYCLE));
        telemetry.addData("CONE_CYCLE_START_TIME:", stateMap.get(constants.CONE_CYCLE_START_TIME));
        telemetry.addData("CYCLE_LIFT_DOWN:", stateMap.get(constants.CYCLE_LIFT_DOWN));
        telemetry.addData("CYCLE_GRABBER:", stateMap.get(constants.CYCLE_GRABBER));
        telemetry.addData("CYCLE_LIFT_UP:", stateMap.get(constants.CYCLE_LIFT_UP));


        if(((String)stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS)){
            coneCycle();
        } else {
            lift.setState();
           // turret.setState((String) stateMap.get(turret.SYSTEM_NAME), lift);
            arm.setState((String) stateMap.get(arm.SYSTEM_NAME));
            grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME), lift);
        }

    }

    public void coneCycle() {
        if (stateMap.get(constants.CONE_CYCLE_START_TIME) == null) {
            stateMap.put(constants.CONE_CYCLE_START_TIME, System.currentTimeMillis());
        }

        if(startLiftDown()) {
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
            stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_IN_PROGRESS);
        } else if(startGrabberAction()){
            stateMap.put(constants.CYCLE_GRABBER, constants.STATE_IN_PROGRESS);
        } else if(startLiftUp()){
            stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_IN_PROGRESS);
            stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
        } else if(isConeCycleComplete()){
            resetConeCycle();
        }

        setConeCycleSystems();
    }

    public void resetConeCycle() {
        stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CONE_CYCLE_START_TIME, null);
    }
    private void setConeCycleSystems() {
        lift.setState();
        grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME), lift);
        arm.setState((String) stateMap.get(arm.SYSTEM_NAME));
    }

    private long coneCycleStartTime() {
        return (long) stateMap.get(constants.CONE_CYCLE_START_TIME);
    }

    private boolean coneCycleStepTimeExpired(long expirationTime) {
        return (expirationTime + coneCycleStartTime()) < System.currentTimeMillis();
    }

    private boolean startLiftUp() {
        return ((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_NOT_STARTED);
    }

    private boolean startLiftDown() {
        return (coneCycleStepTimeExpired(constants.GRABBER_CYCLE_TIME) &&
                ((String) stateMap.get(constants.CONE_CYCLE)).equalsIgnoreCase(constants.STATE_IN_PROGRESS) &&
                ((String)(stateMap.get(constants.CYCLE_LIFT_DOWN))).equalsIgnoreCase(constants.STATE_NOT_STARTED));
    }

    private boolean startGrabberAction() {
        return ((String) stateMap.get(constants.CYCLE_LIFT_DOWN)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_NOT_STARTED);
    }

    private boolean isConeCycleComplete() {
        return (((String) stateMap.get(constants.CYCLE_LIFT_DOWN)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_GRABBER)).equalsIgnoreCase(constants.STATE_COMPLETE) &&
                ((String) stateMap.get(constants.CYCLE_LIFT_UP)).equalsIgnoreCase(constants.STATE_COMPLETE));
    }

    // Just to be safe, the 1+5lowpole drop cone sequence is preserved
    public void dropConeLowPole() throws InterruptedException {
        lift.raiseHeightTo(lift.getPosition() - 150);
        sleep(200); // was 100
        grabber.grabberOpenWide();
    }

    public void dropCone() throws InterruptedException {
        lift.raiseHeightTo(lift.getPosition() - 300);
        sleep(100); // was 100
        grabber.grabberOpen();
        sleep(100); // was 100

        // move away from the pole so the grabber does not hit the pole when swinging back
        // Arm extends with the lower numbers, retracts with higher numbers
        arm.extendHome();
        sleep(200);
    }


    public void pickupCone() throws InterruptedException {
        grabber.grabberClose();
        sleep(250);
//        lift.goToClear();
        lift.raiseHeightTo(lift.getPosition() + 90);
        sleep(300); // original was 200
    }
}

