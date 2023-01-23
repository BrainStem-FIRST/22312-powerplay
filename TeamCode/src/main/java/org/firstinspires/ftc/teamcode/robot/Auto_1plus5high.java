package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Vision.imagecv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.autoclasses.BrainStemRobotA;
import org.firstinspires.ftc.teamcode.robot.autoclasses.ConstantsA;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Config
@Autonomous(name="Robot: Auto 1+5 at High", group="Robot")
public class Auto_1plus5high extends LinearOpMode {
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
    private double TIME_TO_PARK = 27;  //25

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

    Pose2d startingPose, depositPose, pickupPose, parkingPose;
    Pose2d cornerPose;

    // Determine waypoints based on Alliance and Orientation
    double  XFORM_X, XFORM_Y;
    double  pickupDeltaX, pickupDeltaY,
            cornerDeltaX, cornerDeltaY;
    double  startingHeading, startingTangent,
            depositHeading, depositTangent,
            pickupHeading, pickupTangent,
            cornerHeading, cornerTangent;
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
                .addTemporalMarker(0.2, () -> {
                    // lift off the ground for transportation
                    robot.lift.goToClear();
                })

                .setTangent(Math.toRadians(startingTangent))
                .lineToConstantHeading(new Vector2d(cornerPose.getX(), cornerPose.getY()),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75), //3.5),
                        SampleMecanumDrive.getAccelerationConstraint(90))

                .splineToSplineHeading(depositPose, Math.toRadians(depositTangent),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75), //3.5),
                        SampleMecanumDrive.getAccelerationConstraint(90))

                // Timer is from start of the trajectory; it is not an offset
                .addTemporalMarker(1.5, () -> {
                    robot.lift.goToHighPoleHeight();
                })

                .addTemporalMarker(1.77, () -> {
                    robot.turret.goToDepositPosition();
                })

                .addTemporalMarker(2.2, () -> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
                })


                // Start trajectory ends with holding the cone above the pole
                .build();

        return trajectoryStart;
    }

    TrajectorySequence buildDepositTrajectory(BrainStemRobotA robot) {

        TrajectorySequence trajectoryDeposit;

        trajectoryDeposit = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                // This is an empty trajectory with just timer to operate the Gulliver's Tower
                .lineToSplineHeading(depositPose,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), 9.75),
                        SampleMecanumDrive.getAccelerationConstraint(90))

                // Cone picked up outside of the trajectory. Cone is at hand at clearing height
                // Pull arm in for travelling
                .addTemporalMarker(0,()->{
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_HOME);
                })

                .waitSeconds(1.0)

                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.lift.goToHighPoleHeight();
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.turret.goToDepositPosition();
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
                })

                // Start moving turret first, and then lift to avoid kicking the stack
//                .addTemporalMarker(1.0, () -> {
//                    robot.turret.goToDepositPosition();
//                })
//
//                .addTemporalMarker(0.8, () -> {
//                    robot.lift.goToHighPoleHeight();
//                })

                // Clear the pole before adjusting height. Lift move trails the turret move

//                .addTemporalMarker(1.0,()-> {
//                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_DEPOSIT);
//                })

                // ConeCycle will be outside of trajectory
                .build();

        return trajectoryDeposit;
    }

    TrajectorySequence buildPickupTrajectory(BrainStemRobotA robot) {

        TrajectorySequence trajectoryPickup;

        trajectoryPickup = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                // Move to the cone stack head first, stop at arm's reach
                .lineToSplineHeading(pickupPose,
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 9.75),
                        SampleMecanumDrive.getAccelerationConstraint(90))

                // Cone dropped prior to this trajectory.
                // All that is needed to move the tower to the pickup location starting with turret first
                // and then delay-start lift
                .addTemporalMarker(0,()->{
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_HOME);
                })

                .addTemporalMarker(0.1,()->{
                    robot.turret.goToPickupPosition();
                    robot.grabber.grabberOpenWide();
                })

                // Clear the pole before adjusting height. Lift move trails the turret move
                .addTemporalMarker(0.3,()-> {
                    robot.lift.goToPickupHeight();
                })

                // Reach arm to touch the cone
                .addTemporalMarker(0.4,()-> {
                    robot.arm.extendTo(robot.arm.EXTENSION_POSITION_PICKUP);
                })

                // Stop at the pickup position. Cone will be picked up outside of trajectory
                .build();

        return trajectoryPickup;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //------------------------------------------------------
        //                 Initialize the robot
        //------------------------------------------------------

        BrainStemRobotA robot = new BrainStemRobotA(hardwareMap, telemetry, stateMap);

        // this variable is used to calculate liftPositionPickup for stacked cones
        robot.lift.numCyclesCompleted = 0;
        robot.lift.updateLiftPickupPosition();

        robot.lift.resetEncoders();
        robot.turret.resetEncoders();

        // Open grabber to allow Driver to load the initial cone
        robot.grabber.grabberOpen();

        // Not using statemap in this version
//        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
//        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
//        stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
//        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
//        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        // Wait until Alliance and Orientation is set by drivers
        while (!programConfirmation && !isStopRequested()) {
            setProgram();
        }

        robot.grabber.grabberClose();

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

        startingTangent = 90;

        // Determine trajectory headings for all alliance combinations
        if (isAllianceRED) {
            if (isOrientationLEFT) { // RED-LEFT
                XFORM_X = -1;
                XFORM_Y = -1;

                startingHeading = -90;
                startingTangent = startingTangent; //120

                cornerHeading = 180;
                cornerTangent = 90;

                depositHeading = 180;
                depositTangent = 0;

                pickupHeading = 180;
                pickupTangent = 180;

                robot.turret.turret_PICKUP_POSITION_VALUE   = 0;
                robot.turret.turret_DEPOSIT_POSITION_VALUE  = 135;  //245

                robot.arm.EXTENSION_POSITION_PICKUP = 0;
                robot.arm.EXTENSION_POSITION_DEPOSIT = 0.45;  //0.72

                cornerDeltaX = 0;
                cornerDeltaY = 0;

                pickupDeltaX = 0; // previously 0
                pickupDeltaY = 0;  // previously 0
            }
            else {                  // RED-RIGHT TODO: adjust for different quadrant
                XFORM_X = 1;
                XFORM_Y = -1;

                startingHeading = -90;
                startingTangent = 180 - startingTangent; //60

                cornerHeading = -90;
                cornerTangent = 90;

                pickupHeading = -90;
                pickupTangent = 90;

                robot.turret.turret_PICKUP_POSITION_VALUE   = -245;
                robot.turret.turret_DEPOSIT_POSITION_VALUE  = 87;

                cornerDeltaX = 0;
                cornerDeltaY = 0;

                pickupDeltaX = -0.5; //1
                pickupDeltaY = -1;
            }
        }
        else {
            if (isOrientationLEFT) { // BLUE-LEFT TODO: adjust for different quadrant
                XFORM_X = 1;
                XFORM_Y = 1;

                startingHeading = 90;
                startingTangent = (180 - startingTangent) * -1; //-60

                cornerHeading = 90;
                cornerTangent = -90;

                pickupHeading = 90;
                pickupTangent = -90;

                robot.turret.turret_PICKUP_POSITION_VALUE   = 245;
                robot.turret.turret_DEPOSIT_POSITION_VALUE  = -105;

                cornerDeltaX = 0;
                cornerDeltaY = 0;

                pickupDeltaX = 0;
                pickupDeltaY = -0.5;
            }
            else {                  // BLUE-RIGHT TODO: adjust for different quadrant
                XFORM_X = -1;
                XFORM_Y = 1;

                startingHeading = 90;
                startingTangent = startingTangent * -1; //240

                cornerHeading = 90;
                cornerTangent = -90;

                pickupHeading = 90;
                pickupTangent = -90;

                robot.turret.turret_PICKUP_POSITION_VALUE   = -255;
                robot.turret.turret_DEPOSIT_POSITION_VALUE  = 87;

                cornerDeltaX = 2;
                cornerDeltaY = 0;

                pickupDeltaX = 0.5;
                pickupDeltaY = -1;
            }
        }

        telemetry.clearAll();
        telemetry.addLine("Load Cone.  Driver 1 Hit A.");
        telemetry.update();

        // Load the initial cone

        while(!gamepad1.a && !isStopRequested()) {}

        // grab the cone
        robot.grabber.grabberClose();
        sleep(500);


        // Determine trajectory segment positions based on Alliance and Orientation
        startingPose    = new Pose2d(XFORM_X * 36, XFORM_Y * 63.75, Math.toRadians(startingHeading));
        cornerPose      = new Pose2d(XFORM_X * (36 + cornerDeltaX), XFORM_Y * (40 + cornerDeltaY), Math.toRadians(cornerHeading));
        depositPose     = new Pose2d(XFORM_X * (18 + cornerDeltaX), XFORM_Y * (12 + cornerDeltaY), Math.toRadians(depositHeading)); //depositX-28
        pickupPose      = new Pose2d(XFORM_X * (54 + pickupDeltaX), XFORM_Y * (11 + pickupDeltaY), Math.toRadians(pickupHeading));
        parkingPose     = new Pose2d(); // to be defined after reading the signal cone

        robot.drive.setPoseEstimate(startingPose);  // Needed to be called once before the first trajectory

        // Build trajectory sequences before Start signal
        TrajectorySequence startTrajectory   = buildStartTrajectory(robot);
        TrajectorySequence pickupTrajectory  = buildPickupTrajectory(robot);
        TrajectorySequence depositTrajectory = buildDepositTrajectory(robot);
        Trajectory trajectoryPark;

        telemetry.clearAll();
        telemetry.addLine("Load Cone.  Driver 1 Hit A.");
        telemetry.update();

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
                        parkingPose = new Pose2d (-58.75, -11.75, Math.toRadians(-90));
                        break;

                    case 2:
                        parkingPose = new Pose2d (-35.25, -11.75, Math.toRadians(-90));
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
                        parkingPose = new Pose2d (35.25, -11.75, Math.toRadians(-90));
                        break;

                    case 3:
                        parkingPose = new Pose2d (58.75, -11.75, Math.toRadians(-90));
                        break;
                }
        } else {
            if(isOrientationLEFT)   // BLUE-LEFT
                switch (PARKING_NUMBER) {
                    case 1:
                        parkingPose = new Pose2d (58.75, 11.75, Math.toRadians(90));
                        break;

                    case 2:
                        parkingPose = new Pose2d (35.25, 11.75, Math.toRadians(90));
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
                        parkingPose = new Pose2d (-35.25, 11.75, Math.toRadians(90));
                        break;

                    case 3:
                        parkingPose = new Pose2d (-58.75, 11.75, Math.toRadians(90));
                        break;
                }
        }


        // initiate first trajectory asynchronous (go to pickup location) at the start of autonomous
        // Need to call drive.update() to make things move within the loop
        currentTrajectoryState = TrajectoryState.TRAJECTORY_START_STATE;
        robot.drive.followTrajectorySequenceAsync(startTrajectory);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentTrajectoryState) {
                case TRAJECTORY_START_STATE:
                    // Switch to trajectoryPickup once the starting trajectory is complete
                    if (!robot.drive.isBusy()) {
                        // Initial trajectory completed, drop the cone
                        robot.dropCone();
                        sleep(100); // wait for cone to drop

                        // Start the next state
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_PICKUP_STATE;
                        robot.drive.followTrajectorySequenceAsync(buildPickupTrajectory(robot));
                    }
                    break;

                case TRAJECTORY_DEPOSIT_STATE:
                    // Switch to Pickup once the deposit trajectory is complete
                    if (!robot.drive.isBusy()) {
                        // Deposit trajectory completed, drop the cone
                        robot.dropCone();
                        sleep(100); // wait for cone to drop

                        // Continue the cycle until no more cones left
                        if(robot.lift.numCyclesCompleted < 5) {
                            // Start the pickup state
                            currentTrajectoryState = TrajectoryState.TRAJECTORY_PICKUP_STATE;
                            robot.drive.followTrajectorySequenceAsync(buildPickupTrajectory(robot));
                        }
                        else {
                            // the last cone was deposited, go to park.
                            currentTrajectoryState = TrajectoryState.TRAJECTORY_PARKING_STATE;
                        }
                    }

                    break;

                case TRAJECTORY_PICKUP_STATE:
                    // Switch to trajectoryDeposit once the pickup trajectory is complete
                    if (!robot.drive.isBusy()) {
                        // Pickup trajectory completed, pick the cone up
                        robot.grabber.grabberClose();
                        sleep(200); // wait for servo to grab
                        robot.lift.goToClear();
                        sleep(100); // wait for lift to clear the stack

                        // Increase number of cones delivered from the stack. This is used to calculate the lift position when returned back to the stack
                        robot.lift.numCyclesCompleted++;
                        robot.lift.updateLiftPickupPosition();

                        // Start the next state
                        currentTrajectoryState = TrajectoryState.TRAJECTORY_DEPOSIT_STATE;
                        robot.drive.followTrajectorySequenceAsync(buildDepositTrajectory(robot));
                    }

                    break;

                case TRAJECTORY_PARKING_STATE:

                    robot.arm.extendHome();
                    robot.grabber.grabberOpen();
                    robot.turret.moveTo(robot.turret.CENTER_POSITION_VALUE);

                    trajectoryPark = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(parkingPose)
                            .build();
                    robot.drive.followTrajectory(trajectoryPark); // This is synchronous trajectory; code does not advance until the trajectory is complete

                    robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_RESET);

                    currentTrajectoryState = TrajectoryState.TRAJECTORY_IDLE;

                    break;

                case TRAJECTORY_IDLE:
                    // Lower the lift closer to the ground
                    // Do nothing. This concludes the autonomous program
                    break;
            }

//            telemetry.addData("remaining time:", (30.0 - autoTime.seconds()));

            // Is it time to park?
            if (autoTime.seconds() > TIME_TO_PARK &&
                    (currentTrajectoryState != TrajectoryState.TRAJECTORY_PARKING_STATE) &&
                    currentTrajectoryState != TrajectoryState.TRAJECTORY_IDLE) {
                //if((trajectoryPickup.duration()+trajectoryParkFromPickup.duration()) < (30.0 - autoTime.seconds())) {
                currentTrajectoryState = TrajectoryState.TRAJECTORY_PARKING_STATE;
            }
            else {

                telemetry.addData("State:",currentTrajectoryState);
                telemetry.addData("Lift Position=",robot.lift.getPosition());
                telemetry.addData("Turret Position=", robot.turret.getPosition());

                telemetry.update();

                // Continue executing trajectory following
                robot.drive.update();
//                robot.turret.setTurretPower();

                // Execute systems based on stateMap
//                robot.updateSystems();
            }

        }
    }

    void coneCycle(BrainStemRobotA robot) {

//        stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS);
//        while (stateMap.get(constants.CONE_CYCLE).equals(constants.STATE_IN_PROGRESS) && opModeIsActive()) {
//            robot.updateSystems();
//        }
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