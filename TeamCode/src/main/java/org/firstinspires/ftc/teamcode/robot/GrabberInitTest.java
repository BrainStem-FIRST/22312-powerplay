package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: GrabberInitTest", group="Robot")
public class GrabberInitTest extends LinearOpMode {
    public void runOpMode(){
        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        robot.updateSystems();
        waitForStart();
    }

}
