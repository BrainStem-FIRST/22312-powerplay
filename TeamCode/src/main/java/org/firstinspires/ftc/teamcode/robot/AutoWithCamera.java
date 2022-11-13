package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.auto.imagecv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


@Config
@Autonomous(name="Robot: Test Auto with Camera", group="Robot")
public class AutoWithCamera extends LinearOpMode {
    //camera
    public OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    public int Ending_Location = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 1000;
    double cx = 100;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.00037;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null; //camera

    //original auto
    public static int PARKING_NUMBER = 1; // Controlled by the dashboard for test purposes
    public static double SPEED = 50.0;    // Controlled by the dashboard for test purposes

    // Alliance variables
    public static boolean isAllianceRED = true;
    public static boolean isOrientationLEFT = true;

    private ElapsedTime autoTime = new ElapsedTime();
    private double TIME_TO_PARK = 25.0;

    Constants constants = new Constants();

    // Trajectory related variables
    enum    TrajectoryState {
        TRAJECTORY_START_STATE,
        TRAJECTORY_REPEAT_STATE,
        TRAJECTORY_PARKING_STATE,
        TRAJECTORY_IDLE
    }
    TrajectoryState currentTrajectoryState = TrajectoryState.TRAJECTORY_START_STATE;


    @Override
    public void runOpMode() {


        //------------------------------------------------------
        //                 Initialize the robot
        //------------------------------------------------------

        //may or may not put state maps here
        Map<String, String> stateMap = new HashMap<String, String>() {
        };

        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);

        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.lift.numCyclesCompleted = 0;

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);


        //------------------------------------------------------
        //            Variable used for trajectories
        //------------------------------------------------------

        // For initial testing purposes, assume the robot is in RED ALLIANCE and
        // the starting position is on LEFT
        //
        // Need a variable to determine the association with alliance and
        // adjust the coordinate system accordingly.

        Pose2d startingPose, pickupPose, deliverPose, parkingPose;
        TrajectorySequence trajectory1, trajectory2, trajectoryPark;

        // Determine waypoints based on Alliance and Orientation
        double XFORM_X = -1;
        double XFORM_Y = -1;
        double startingHeading = 90;
        double deliveryHeading = 180;
        double pickupHeading = 0;

        double CONE_CYCLE_DURATION = 1.0;   // seconds to allow cone cycle to complete before moving again in trajectory

        //camera initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam-1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50); //camera


        // Determine trajectory headings for all alliance combinations
        if (isAllianceRED) {
            if (isOrientationLEFT) { // RED-LEFT
                XFORM_X = -1;
                XFORM_Y = -1;
                startingHeading = 90;
                pickupHeading = 180;
                deliveryHeading = 180;
            } else {                  // RED-RIGHT
                XFORM_X = 1;
                XFORM_Y = -1;
                startingHeading = 90;
                pickupHeading = 0;
                deliveryHeading = 0;
            }
        } else {
            if (isOrientationLEFT) { // BLUE-LEFT
                XFORM_X = 1;
                XFORM_Y = 1;
                startingHeading = -90;
                pickupHeading = 0;
                deliveryHeading = 0;
            } else {                  // BLUE-RIGHT
                XFORM_X = -1;
                XFORM_Y = 1;
                startingHeading = -90;
                pickupHeading = 180;
                deliveryHeading = 180;
            }
        }

        // Determine trajectory segment positions based on Alliance and Orientation
        startingPose    = new Pose2d(XFORM_X * 36, XFORM_Y * 67.25, Math.toRadians(startingHeading));
        pickupPose      = new Pose2d(XFORM_X * 64.25, XFORM_Y * 15.5, Math.toRadians(pickupHeading));
        deliverPose     = new Pose2d(XFORM_X * 0, XFORM_Y * 15.5, Math.toRadians(deliveryHeading));
        parkingPose = new Pose2d();

        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        /****************  define trajectories  *********************/

        // From starting position, move forward and spline to pickup position,
        // then perform one iteration of pickup/delivery cycle
        trajectory1 = robot.drive.trajectorySequenceBuilder(startingPose)
                //moves forward in a line facing 90 degrees away (positioned in between two poles)
                .forward(41)
                .addTemporalMarker(0.5,()->{    // Start positioning scaffolding half a second into trajectory
                    // Position grabber to the front of the robot
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_MEDIUM);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
                })
                // Drop off the cone at hand on the Medium High pole on the right
                // This action starts after .forward() is completed
                .addTemporalMarker(()->stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS))
                .waitSeconds(CONE_CYCLE_DURATION)   // wait for the cone cycle to complete
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE))
                .forward(3)

                .addTemporalMarker(()->{
                   // Position grabber to the front of the robot
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })
                .splineTo(new Vector2d(pickupPose.getX(), pickupPose.getY()), pickupPose.getHeading())
                // Trajectory ends once it reaches the pickup location; it does NOT pick up cone in this trajectory
                .build();

        // Starting from the pickup position, pick up cone -> run to delivery pose -> drop cone -> run to pickup pose
        // This trajectory is intended to be repeated
        trajectory2 = robot.drive.trajectorySequenceBuilder(trajectory1.end())
                // Once at the pickup location, initiate cone cycle
                .addTemporalMarker(()->stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS))
                .waitSeconds(CONE_CYCLE_DURATION)   // wait for the cone cycle to complete

                .setReversed(true)  // go backwards

                // Lift to high pole while running backwards
                // TODO: Adjust offset time so the action completes right before the robot reaches High Pole
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
                })
                .lineToLinearHeading(deliverPose)
                // Once at the delivery location, initiate the cone cycle again
                .addTemporalMarker(()->stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS))
                .waitSeconds(CONE_CYCLE_DURATION)   // wait for the cone cycle to complete as there is nothing else left in this trajectory

                .setReversed(false) // go forwards

                // shift position of lift and turret while running to pickup position
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                })
                // Go back for a new cone
                .splineTo(new Vector2d(pickupPose.getX(), pickupPose.getY()), pickupPose.getHeading())

                // Stop at the pickup position so the loop can identify when the trajectory sequence completed by a call to drive.isBusy()
                .build();



        /********************* Initialization Complete **********************/

        robot.grabber.grabberClose();
        telemetry.addData("Grabber Position", "%f", robot.grabber.grabberPosition());
        telemetry.update();

        // waitForStart();
        //camera
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == RIGHT || tag.id == MIDDLE) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\n But we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            // Couldn't read the tag, take a guess for parking spot
            PARKING_NUMBER = 2;
        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */
            // Assign PARKING_NUMBER based on tagOfInterest
            // Determine parking position based on Alliance and Orientation

            if(isAllianceRED) {
                if(isOrientationLEFT)   // RED-LEFT
                    switch (PARKING_NUMBER) {
                        case 1:
                            parkingPose = new Pose2d (-60, -12, Math.toRadians(180));
                            break;

                        case 2:
                            parkingPose = new Pose2d (-36, -12, Math.toRadians(180));
                            break;

                        case 3:
                            parkingPose = new Pose2d (-12, -12, Math.toRadians(-90));
                            break;
                    }
                else                    // RED-RIGHT
                    switch (PARKING_NUMBER) {
                        case 1:
                            parkingPose = new Pose2d (12, -12, Math.toRadians(-90));
                            break;

                        case 2:
                            parkingPose = new Pose2d (36, -12, Math.toRadians(0));
                            break;

                        case 3:
                            parkingPose = new Pose2d (60, -12, Math.toRadians(0));
                            break;
                    }
            }
            else {
                if(isOrientationLEFT)   // BLUE-LEFT
                    switch (PARKING_NUMBER) {
                        case 1:
                            parkingPose = new Pose2d (60, 12, Math.toRadians(0));
                            break;

                        case 2:
                            parkingPose = new Pose2d (36, 12, Math.toRadians(0));
                            break;

                        case 3:
                            parkingPose = new Pose2d (12, 12, Math.toRadians(90));
                            break;
                    }
                else                    // BLUE-RIGHT
                    switch (PARKING_NUMBER) {
                        case 1:
                            parkingPose = new Pose2d (-12, 12, Math.toRadians(90));
                            break;

                        case 2:
                            parkingPose = new Pose2d (-36, 12, Math.toRadians(-90));
                            break;

                        case 3:
                            parkingPose = new Pose2d (-60, 12, Math.toRadians(-90));
                            break;
                    }
            }

        }

        trajectoryPark = robot.drive.trajectorySequenceBuilder(trajectory2.end())
                .splineTo(new Vector2d(parkingPose.getX(), parkingPose.getY()), parkingPose.getHeading())
                .build();

        // initiate first trajectory asynchronous (go to pickup location) at the start of autonomous
        // Need to call drive.update() to make things move within the loop
        robot.drive.followTrajectorySequenceAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentTrajectoryState) {
                case TRAJECTORY_START_STATE:
                    // Switch to next trajectory once the starting trajectory is complete
                    if (!robot.drive.isBusy()) {
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_REPEAT_STATE;
                        robot.drive.followTrajectorySequenceAsync(trajectory2);
                        //currentTrajectoryState = TrajectoryState.TRAJECTORY_IDLE;
                    }
                    break;

                case TRAJECTORY_REPEAT_STATE:
                    // Re-invoke trajectory2 when it finishes. Exit this state only when it's time to park.

                    if (!robot.drive.isBusy()) {
                        // Trajectory is complete. Either re-start it or go park
                        robot.lift.numCyclesCompleted += 1;

                        // Is it time to park?
                        if (autoTime.seconds() > TIME_TO_PARK) {
                            currentTrajectoryState = TrajectoryState.TRAJECTORY_PARKING_STATE;
                            robot.drive.followTrajectorySequenceAsync(trajectoryPark);
                        } else {
                            // repeat the cycle
                            robot.drive.followTrajectorySequenceAsync(trajectory2);
                        }
                    }
                    break;

                case TRAJECTORY_PARKING_STATE:
                    // Wait for the robot to complete the trajectory and go to IDLE state
                    if (!robot.drive.isBusy()) {
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_IDLE;
                    }
                    break;

                case TRAJECTORY_IDLE:
                    // Do nothing. This concludes the autonomous program
                    break;
            }

            // Execute systems based on stateMap
            robot.updateSystems();
            telemetry.addData("Grabber Position", "%f", robot.grabber.grabberPosition());

            // Continue executing trajectory following
            robot.drive.update();

            telemetry.addData("Cycle:", "%d", robot.lift.numCyclesCompleted);
            telemetry.update();
        }
    }

    //camera
    void tagToTelemetry(AprilTagDetection detection) {
        Ending_Location = detection.id;
        telemetry.addData("Thing :", Ending_Location);
        telemetry.update();
    }//camera
}