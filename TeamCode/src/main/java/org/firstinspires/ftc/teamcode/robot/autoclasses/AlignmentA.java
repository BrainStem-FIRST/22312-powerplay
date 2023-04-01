package org.firstinspires.ftc.teamcode.robot.autoclasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class AlignmentA {
    private Telemetry telemetry;

    public ServoImplEx alignment;

    // Servo Positions TODO: find all positions
    public final double ALIGNMENT_POSITION_UP     = 1.0;      // above grabber
    public final double ALIGNMENT_POSITION_DOWN   = 0;    // below grabber

    public AlignmentA(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        alignment = (ServoImplEx) hwMap.servo.get("Alignment");

        // Scale the operating range of Servos and set initial position
        // lower boundary for alignUp; higher boundary for alignDown
        alignment.setPwmRange(new PwmControl.PwmRange(465,1760));
//        alignment.setDirection(ServoImplEx.Direction.REVERSE);
        alignUp();

    }

    public void alignUp() {
        alignment.setPosition(ALIGNMENT_POSITION_UP);
    }

    public void alignDown() {
        alignment.setPosition(ALIGNMENT_POSITION_DOWN);
    }
}
