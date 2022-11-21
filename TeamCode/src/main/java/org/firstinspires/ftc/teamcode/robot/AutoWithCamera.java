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

    // Corresponds to parking zones
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null; //camera

    // Trajectory related variables
    public static boolean isAllianceRED = true; //TODO: dynamically update values
    public static boolean isOrientationLEFT = true;

    // original auto
    public static int PARKING_NUMBER = 1; // Controlled by the dashboard for test purposes
    public static double SPEED = 50.0;    // Controlled by the dashboard for test purposes
    private ElapsedTime autoTime = new ElapsedTime();
    private double TIME_TO_PARK = 50.0;

    // used for trajectory state machine
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

        Constants constants = new Constants();
        Map<String, String> stateMap = new HashMap<String, String>() { };
        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);

        // this variable is used to calculate liftPositionPickup for stacked cones
        robot.lift.numCyclesCompleted = 0;

        robot.lift.resetEncoders();

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);


        //------------------------------------------------------
        //            Camera initialization
        //------------------------------------------------------

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


        //------------------------------------------------------
        //            Variable used for trajectories
        //------------------------------------------------------

        Pose2d startingPose, pickupPose, deliverPose, parkingPose;
        TrajectorySequence trajectory1, trajectory2, trajectoryPark;

        // Determine waypoints based on Alliance and Orientation
        double XFORM_X, XFORM_Y;
        double startingHeading, deliveryHeading, pickupHeading;

        double CONE_CYCLE_DURATION = 1.5;   // seconds to allow cone cycle to complete before moving again in trajectory


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
        }
        else {
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
        pickupPose      = new Pose2d(XFORM_X * 65, XFORM_Y * 12, Math.toRadians(pickupHeading));
        deliverPose     = new Pose2d(XFORM_X * 0, XFORM_Y * 12, Math.toRadians(deliveryHeading));
        parkingPose = new Pose2d();

        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        //------------------------------------------------------
        //            Define trajectories
        //------------------------------------------------------

        // From starting position, move forward, deliver on Low Pole, and spline to pickup position
        // Pickup/delivery is part of Trajectory 2
        trajectory1 = robot.drive.trajectorySequenceBuilder(startingPose)
                //moves forward in a line facing 90 degrees away (positioned in between two poles)
                .forward(41)
                .addTemporalMarker(0.5,()->{    // Start positioning scaffolding half a second into trajectory
                    // Position grabber to the left of the robot
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.EXTEND_LEFT);
                })
                // Drop off the cone at hand on the Low pole on the left
                // This action starts after .forward() is completed
                .addTemporalMarker(()->stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS))
                .waitSeconds(CONE_CYCLE_DURATION)   // wait for the cone cycle to complete
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })
                .forward(3)

                .addTemporalMarker(()->{
                   // Lower lift for travelling
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
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
                // Cone cycle's lift up is not high enough to clear the cone from stack. Lift more to Medium Pole Height
                //.addTemporalMarker(()->robot.lift.raiseHeightTo(255))
                .addTemporalMarker(()->stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POSITION_CLEAR))
                .setReversed(true)  // go backwards

                // Lift to high pole while running backwards
                // TODO: Adjust offset time so the action completes right before the robot reaches High Pole
                .UNSTABLE_addTemporalMarkerOffset(2.0, ()->{
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.EXTEND_LEFT);
                })

                .lineToLinearHeading(deliverPose)

                // Once at the delivery location, initiate the cone cycle again
                .addTemporalMarker(()->{
                    // Deposit cone
                    stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
                    // Increase number of cones delivered from the stack. This is used to calculate the lift position when returned back to the stack
                    robot.lift.numCyclesCompleted++;
                    robot.lift.updateLiftPickupPosition();
                })
                .waitSeconds(CONE_CYCLE_DURATION)   // wait for the cone cycle to complete as there is nothing else left in this trajectory

                .setReversed(false) // go forwards

                // shift position of lift and turret while running to pickup position
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                })

                // Go back for a new cone
                .lineToLinearHeading(pickupPose)

                // Stop at the pickup position so the loop can identify when the trajectory sequence completed by a call to drive.isBusy()
                .build();

        // Load the initial cone
        robot.grabber.grabberClose();

        /********************* Initialization Complete **********************/


        //-------------------------------------------------------------------
        // waitForStart();  // Replaced with the following while loop
        //-------------------------------------------------------------------

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
        autoTime.reset();   // Start your autonomous counter

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
            if(tagOfInterest.id == LEFT)
                PARKING_NUMBER = 1;
            else if (tagOfInterest.id == MIDDLE)
                PARKING_NUMBER = 2;
            else
                PARKING_NUMBER = 3;

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
                .lineToLinearHeading(parkingPose)
                .build();

        // initiate first trajectory asynchronous (go to pickup location) at the start of autonomous
        // Need to call drive.update() to make things move within the loop
        currentTrajectoryState = TrajectoryState.TRAJECTORY_START_STATE;
        robot.drive.followTrajectorySequenceAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentTrajectoryState) {
                case TRAJECTORY_START_STATE:
                    // Switch to next trajectory once the starting trajectory is complete
                    if (!robot.drive.isBusy()) {
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_REPEAT_STATE;
                        robot.drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;

                case TRAJECTORY_REPEAT_STATE:
                    // Re-invoke trajectory2 when it finishes. Exit this state only when it's time to park
                    if (!robot.drive.isBusy()) {
                        // Trajectory is complete. Either re-start it or go park

                        // Is it time to park?
                        if (autoTime.seconds() > TIME_TO_PARK) {
                        //if((trajectory2.duration()+trajectoryPark.duration()) < (30.0 - autoTime.time())) {
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

            // Continue executing trajectory following
            robot.drive.update();

            telemetry.addData("Trajectory State:", currentTrajectoryState);
            telemetry.addData("Cones removed from stack:", robot.lift.numCyclesCompleted);
            telemetry.addData("New Pickup Height:", robot.lift.liftPositionPickup);
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