package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(group = "robot")
public class MotorTest extends LinearOpMode {

    public static int TARGET_TICK = 1000;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        waitForStart();

        if (isStopRequested()) return;

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(TARGET_TICK);
        }

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : motors) {
            motor.setPower(0.2);
        }

        telemetry.addData("FL Tick Count = ", leftFront.getCurrentPosition());
        telemetry.addData("FR Tick Count = ", rightFront.getCurrentPosition());
        telemetry.addData("BL Tick Count = ", leftRear.getCurrentPosition());
        telemetry.addData("BR Tick Count = ", rightRear.getCurrentPosition());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

}