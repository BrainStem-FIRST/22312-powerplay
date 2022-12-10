package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Vision.imagecv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Config
@Autonomous(name="Robot: Auto", group="Robot")
public class Auto extends LinearOpMode {
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
    public static boolean isAllianceRED;
    private boolean allianceIsSet = false;
    public static boolean isOrientationLEFT;
    private boolean orientationIsSet = false;
    private boolean programConfirmation = false;

    // original auto
    public static int PARKING_NUMBER = 2; // Controlled by the dashboard for test purposes
    public static double SPEED = 60.0;    // Controlled by the dashboard for test purposes
    private ElapsedTime autoTime = new ElapsedTime();
    private double TIME_TO_PARK = 50;  //25

    // used for trajectory state machine
    enum    TrajectoryState {
        TRAJECTORY_START_STATE,
        TRAJECTORY_DEPOSIT_STATE,
        TRAJECTORY_PICKUP_STATE,
        TRAJECTORY_PARKING_STATE,
        TRAJECTORY_IDLE
    }
    TrajectoryState currentTrajectoryState = TrajectoryState.TRAJECTORY_START_STATE;

    Map<String, String> stateMap = new HashMap<String, String>() { };
    ConstantsA constants = new ConstantsA();


    //------------------------------------------------------
    //            Variable used for trajectories
    //------------------------------------------------------

    Pose2d startingPose, pickupPose, depositPose, parkingPose;
    Pose2d cornerPose;
    TrajectorySequence trajectoryParkFromDeposit, trajectoryParkFromPickup;

    // Determine waypoints based on Alliance and Orientation
    double  XFORM_X, XFORM_Y;
    double  pickupDeltaX, pickupDeltaY,
            depositDeltaX, depositDeltaY,
            preloadDeltaX, preloadDeltaY,
            cornerDeltaX, cornerDeltaY;
    double  startingHeading, deliveryHeading, pickupHeading;
    String  turretState, armState;

    // Build trajectories

    //------------------------------------------------------
    //            Define trajectories
    //------------------------------------------------------

    // From starting position, turn sideways (to push the signal cone), strafe, deliver on Low Pole, spline to pickup position, and pickup top-most cone

    TrajectorySequence buildStartTrajectory(BrainStemRobotA robot) {
        TrajectorySequence trajectoryStart;

        trajectoryStart = robot.drive.trajectorySequenceBuilder(startingPose)

                //make sure the cone is lifted a little from the ground before robot starts moving

                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(cornerPose,Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(180), 3.5),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .setTangent(Math.toRadians(90))
                .addTemporalMarker(()->{
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .lineToLinearHeading(pickupPose) //,Math.toRadians(60))

                .addTemporalMarker(3.0, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                    stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.DEPOSIT_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
                    robot.grabber.grabberClose();
                })

                .addTemporalMarker(6.5, () -> {
                    coneCycle(robot);
                })

                .build();

        return trajectoryStart;
    }

    TrajectorySequence buildDepositTrajectory(BrainStemRobotA robot) {

        TrajectorySequence trajectoryDeposit;

        trajectoryDeposit = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                // Lift to high pole while running backwards

                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_MEDIUM);
                    robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_MIDPOLE);
                    stateMap.put(robot.turret.SYSTEM_NAME, turretState);
                    stateMap.put(robot.arm.SYSTEM_NAME, armState);
                })

                .lineToLinearHeading(depositPose,
                        SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))

                .build();

        return trajectoryDeposit;
    }

    TrajectorySequence buildPickupTrajectory(BrainStemRobotA robot) {

        TrajectorySequence trajectoryPickup;

        trajectoryPickup = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                // shift position of lift and turret while running to pickup position
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
                    robot.lift.raiseHeightTo(robot.lift.liftPositionPickup);
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                })

                // Go back for a new cone
                .lineToLinearHeading(pickupPose,
                        SampleMecanumDrive.getVelocityConstraint(SPEED, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))

                // Stop at the pickup position so the loop can identify when the trajectory sequence completed by a call to drive.isBusy()
                .build();

        return trajectoryPickup;
    }

    public double xform_Rad(double degree) {
        double rad= Math.toRadians(degree);

        // Determine trajectory headings for all alliance combinations
        if (isAllianceRED) {
            if (isOrientationLEFT) {    // RED-LEFT
                // No change to rad
            }
            else {                      // RED-RIGHT
                // mirror on y axis
            }
        }
        else {
            if (isOrientationLEFT) {    // BLUE-LEFT
                // mirror on x axis AND y axis
            }
            else {                      // BLUE-RIGHT
                // mirror on x axis ONLY
            }
        }
        return rad;
    }



    @Override
    public void runOpMode() throws InterruptedException {

        //------------------------------------------------------
        //                 Initialize the robot
        //------------------------------------------------------

        BrainStemRobotA robot = new BrainStemRobotA(hardwareMap, telemetry, stateMap);

        // this variable is used to calculate liftPositionPickup for stacked cones
        robot.lift.numCyclesCompleted = 0;

        robot.lift.resetEncoders();
        robot.turret.resetEncoders();

        // Open grabber to allow Driver to load the initial cone
        robot.grabber.grabberOpen();

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        // Wait until Alliance and Orientation is set by drivers
        while (!programConfirmation && !isStopRequested()) {
            setProgram();
        }


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


        double CONE_CYCLE_DURATION = 1.0;   // seconds to allow cone cycle to complete before moving again in trajectory


        // Determine trajectory headings for all alliance combinations
        if (isAllianceRED) {
            if (isOrientationLEFT) { // RED-LEFT
                XFORM_X = -1;
                XFORM_Y = -1;
                startingHeading = 90;
                pickupHeading = 180;
                deliveryHeading = 180;
                turretState = robot.turret.LEFT_POSITION;
                armState = robot.arm.EXTEND_LEFT;
                pickupDeltaX = -2;  // originally 0; adjusted for FLL Champ field
                pickupDeltaY = 0;

                depositDeltaX = -1;
                depositDeltaY = 2;

                preloadDeltaX = -1;
                preloadDeltaY = 1;
            } else {                  // RED-RIGHT
                XFORM_X = 1;
                XFORM_Y = -1;
                startingHeading = 90;
                pickupHeading = 0;
                deliveryHeading = 0;
                turretState = robot.turret.RIGHT_POSITION;
                armState = robot.arm.FULL_EXTEND;
                pickupDeltaX = 2;
                pickupDeltaY = 2;

                depositDeltaX = 0;
                depositDeltaY = -2;

                preloadDeltaX = 0;
                preloadDeltaY = 2;
            }
        }
        else {
            if (isOrientationLEFT) { // BLUE-LEFT
                XFORM_X = 1;
                XFORM_Y = 1;
                startingHeading = -90;
                pickupHeading = 0;
                deliveryHeading = 0;
                turretState = robot.turret.LEFT_POSITION;
                armState = robot.arm.EXTEND_LEFT;
                pickupDeltaX = 0;
                pickupDeltaY = -3;

                depositDeltaX = -1;
                depositDeltaY = 0;

                preloadDeltaX = -0.5;
                preloadDeltaY = 0;
            } else {                  // BLUE-RIGHT
                XFORM_X = -1;
                XFORM_Y = 1;
                startingHeading = -90;
                pickupHeading = 180;
                deliveryHeading = 180;
                turretState = robot.turret.RIGHT_POSITION;
                armState = robot.arm.FULL_EXTEND;
                pickupDeltaX = 2;
                pickupDeltaY = 4;

                depositDeltaX = 0;
                depositDeltaY = -2;

                preloadDeltaX = 0;
                preloadDeltaY = 2;
            }
        }

        // Determine trajectory segment positions based on Alliance and Orientation
        startingPose    = new Pose2d(XFORM_X * 35.5, XFORM_Y * 63.75, Math.toRadians(startingHeading));
        cornerPose      = new Pose2d(XFORM_X * (62 + cornerDeltaX), XFORM_Y * (57 + pickupDeltaY), Math.toRadians(-90));
        pickupPose      = new Pose2d(XFORM_X * (59.6 + pickupDeltaX), XFORM_Y * (12 + pickupDeltaY), Math.toRadians(245));

        depositPose     = new Pose2d(XFORM_X * (24 + depositDeltaX), XFORM_Y * (10 + depositDeltaY), Math.toRadians(deliveryHeading));
        parkingPose     = new Pose2d(); // to be defined after reading the signal cone

        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        telemetry.clearAll();
        telemetry.addLine("Load Cone.  Driver 1 Hit A.");
        telemetry.update();

        // Load the initial cone
        while(!gamepad1.a && !isStopRequested()) {}

        robot.grabber.grabberClose();

        telemetry.clearAll();
        telemetry.addLine("Robot is Ready!");
        telemetry.update();

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
            // Couldn't read the tag, choose whichever tag is harder to read
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
        }

        if(isAllianceRED) {
            if(isOrientationLEFT)   // RED-LEFT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (-58.75, -11.75, Math.toRadians(180));
                        break;

                    case 2:
                        parkingPose = new Pose2d (-35.25, -11.75, Math.toRadians(180));
                        break;

                    case 3:
                        parkingPose = new Pose2d (-11.75, -11.75, Math.toRadians(-90));
                        break;
                }
            else                    // RED-RIGHT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (11.75, -11.75, Math.toRadians(-90));
                        break;

                    case 2:
                        parkingPose = new Pose2d (35.25, -11.75, Math.toRadians(0));
                        break;

                    case 3:
                        parkingPose = new Pose2d (58.75, -11.75, Math.toRadians(0));
                        break;
                }
        } else {
            if(isOrientationLEFT)   // BLUE-LEFT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (58.75, 11.75, Math.toRadians(0));
                        break;

                    case 2:
                        parkingPose = new Pose2d (35.25, 11.75, Math.toRadians(0));
                        break;

                    case 3:
                        parkingPose = new Pose2d (11.75, 11.75, Math.toRadians(90));
                        break;
                }
            else                    // BLUE-RIGHT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (-11.75, 11.75, Math.toRadians(90));
                        break;

                    case 2:
                        parkingPose = new Pose2d (-35.25, 11.75, Math.toRadians(-90));
                        break;

                    case 3:
                        parkingPose = new Pose2d (-58.75, 11.75, Math.toRadians(-90));
                        break;
                }
        }


        Trajectory trajectoryPark;


        // initiate first trajectory asynchronous (go to pickup location) at the start of autonomous
        // Need to call drive.update() to make things move within the loop
        currentTrajectoryState = TrajectoryState.TRAJECTORY_START_STATE;
        robot.drive.followTrajectorySequenceAsync(buildStartTrajectory(robot));

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentTrajectoryState) {
                case TRAJECTORY_START_STATE:
                    // Switch to trajectoryDeposit once the starting trajectory is complete
                    if (!robot.drive.isBusy()) {
                        // Initial trajectory completed

                        // Start the next state
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_PICKUP_STATE;
                        telemetry.addData("lift position:", robot.lift.getPosition());
                        telemetry.addData("turret position", robot.turret.getPosition());

                        sleep(3000); // wait to see positioning
                    }
                    break;

                case TRAJECTORY_DEPOSIT_STATE:
                    // Invoke trajectoryPickup when deposit is complete.
                    // Park when there is not enough time to complete another trajectory.

                    // Deposit cone at delivery station
                    robot.lift.raiseHeightTo(robot.lift.LIFT_CLEAR_HEIGHT);
                    sleep(500);

                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.DEPOSIT_POSITION);
                    stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);

                    while(!robot.turret.getCurrentState().equalsIgnoreCase(robot.turret.DEPOSIT_POSITION) &&
                            !robot.lift.getCurrentState(robot.lift.APPROACH_HEIGHT).equalsIgnoreCase(robot.lift.LIFT_POLE_LOW)) {
                        robot.updateSystems();
                        telemetry.addData("Turret Position (deposit):",robot.turret.getPosition());
                        telemetry.update();
                    }

                    sleep(1000);
                    coneCycle(robot);

                    // Go back to pickup cycle
                    currentTrajectoryState = TrajectoryState.TRAJECTORY_PICKUP_STATE;
                    //robot.drive.followTrajectorySequenceAsync(buildPickupTrajectory(robot));

                    break;

                case TRAJECTORY_PICKUP_STATE:
                    // Invoke trajectoryDeposit when Pickup finishes.
                    // Park when there is not enough time to complete another trajectory.

                    // Deposit cone at pickup station
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.PICKUP_POSITION);
                    stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);

                    while(!robot.turret.getCurrentState().equalsIgnoreCase(robot.turret.PICKUP_POSITION) &&
                            !robot.lift.getCurrentState(robot.lift.APPROACH_HEIGHT).equalsIgnoreCase(robot.lift.LIFT_PICKUP)) {
                        robot.updateSystems();
                        telemetry.addData("Turret Position (pickup):",robot.turret.getPosition());
                        telemetry.update();
                    }
//                    // Manually controlling lift and turret, bypassing updateSystems
//                    robot.turret.moveTo(robot.turret.PICKUP_POSITION_VALUE);
//                    robot.lift.raiseHeightTo(robot.lift.liftPositionPickup);

                    sleep(1000);
                    coneCycle(robot);

                    // Increase number of cones delivered from the stack. This is used to calculate the lift position when returned back to the stack
                    robot.lift.numCyclesCompleted++;
                    robot.lift.updateLiftPickupPosition();

                    // Go back to deposit cycle
                    currentTrajectoryState = TrajectoryState.TRAJECTORY_DEPOSIT_STATE;

                    break;

                case TRAJECTORY_PARKING_STATE:
                    trajectoryPark = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(parkingPose)
                            .build();
                    robot.drive.followTrajectory(trajectoryPark);
                    robot.arm.extendHome();
                    robot.turret.moveTo(robot.turret.CENTER_POSITION_VALUE);
                    robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_GROUND);

                    // Wait for the robot to complete the trajectory and go to IDLE state
                    if (!robot.drive.isBusy()) {
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_IDLE;
                    }
                    break;

                case TRAJECTORY_IDLE:
                    // Lower the lift closer to the ground
                    // Do nothing. This concludes the autonomous program
                    break;
            }

            telemetry.addData("remaining time:", (30.0 - autoTime.seconds()));

            // Is it time to park?
            if (autoTime.seconds() > TIME_TO_PARK &&
                    (currentTrajectoryState != TrajectoryState.TRAJECTORY_PARKING_STATE) &&
                     currentTrajectoryState != TrajectoryState.TRAJECTORY_IDLE) {
                //if((trajectoryPickup.duration()+trajectoryParkFromPickup.duration()) < (30.0 - autoTime.seconds())) {
                currentTrajectoryState = TrajectoryState.TRAJECTORY_PARKING_STATE;
            }
            else {

                // Continue executing trajectory following
                robot.drive.update();

                // Execute systems based on stateMap
                robot.updateSystems();
                // Do all updates except for lift.setState
//                robot.turret.setState((String) stateMap.get(robot.turret.SYSTEM_NAME), robot.lift);
//                robot.arm.setState((String) stateMap.get(robot.arm.SYSTEM_NAME));
//                robot.grabber.setState((String) stateMap.get(robot.grabber.SYSTEM_NAME), robot.lift);
            }

            telemetry.addData("Lift Position=",robot.lift.getPosition());

            telemetry.update();
        }
    }

    void coneCycle(BrainStemRobotA robot) {
        stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
        while (stateMap.get(constants.CONE_CYCLE).equals(constants.STATE_IN_PROGRESS) && opModeIsActive()) {
            robot.updateSystems();
        }
    }
    //camera
    void tagToTelemetry(AprilTagDetection detection) {
        Ending_Location = detection.id;
        telemetry.addData("Thing :", Ending_Location);
        telemetry.update();
    }//camera

    private void setProgram() {
        allianceIsSet = false;
        orientationIsSet = false;

        telemetry.clearAll();
        telemetry.addLine("Set Alliance: Driver 1-> X=Blue or B=Red.");
        telemetry.update();
        while (!allianceIsSet && !isStopRequested()) {
            if (gamepad1.x) {
                isAllianceRED = false;
                allianceIsSet = true;
            } else if (gamepad1.b) {
                isAllianceRED = true;
                allianceIsSet = true;
            }
        }

        String allianceColor;
        if (isAllianceRED) {
            allianceColor = "RED";
        } else {
            allianceColor = "BLUE";
        }

        telemetry.clearAll();
        telemetry.addData("Alliance Set: ", allianceColor);
        telemetry.update();

        sleep(1500);

        telemetry.clearAll();
        telemetry.addLine("Set Orientation: Driver 1-> dpad left or right.");
        telemetry.update();

        while (!orientationIsSet && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                isOrientationLEFT = true;
                orientationIsSet = true;
            } else if (gamepad1.dpad_right) {
                isOrientationLEFT = false;
                orientationIsSet = true;
            }
        }

        String orientation;
        if (isOrientationLEFT) {
            orientation = "LEFT";
        } else {
            orientation = "RIGHT";
        }

        telemetry.addData("Orientation Set: ", orientation);
        telemetry.update();

        sleep(1500);
        telemetry.clearAll();
        telemetry.addLine("Confirm Program:");
        telemetry.addData("Alliance   :", allianceColor);
        telemetry.addData("Orientation:", orientation);
        telemetry.addLine("Driver 2-> A To Confirm. B to Restart.");
        telemetry.update();

        boolean confirmation = false;
        while (!confirmation && !isStopRequested()) {
            telemetry.clearAll();
            if (gamepad2.a) {
                telemetry.clearAll();
                telemetry.addLine("Program Confirmed");
                telemetry.update();
                programConfirmation = true;
                confirmation = true;
            } else if (gamepad2.b) {
                telemetry.clearAll();
                telemetry.addLine("Program Rejected");
                telemetry.update();
                programConfirmation = false;
                orientationIsSet = false;
                allianceIsSet = false;
                confirmation = true;
            }
        }

        sleep(1500);
        telemetry.clearAll();
    }

}