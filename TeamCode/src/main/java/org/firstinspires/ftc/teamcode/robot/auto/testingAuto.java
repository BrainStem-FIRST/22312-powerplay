package org.firstinspires.ftc.teamcode.robot.auto;//package org.firstinspires.ftc.teamcode.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auto.imagecv.AprilTagDetectionPipeline;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Autonomous (name = "testing auto 1")
//public class testingAuto extends LinearOpMode {
//
//    ///////////////////
//    // april tags stuff
//    ///////////////////
//
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//    int Ending_Location = 1;
//    double fx = 578.272;
//    double fy = 1000;
//    double cx = 100;
//    double cy = 221.506;
//    double tagsize = 0.00037;
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//    int robotEndingPose;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        ///////////////////////////
//        // april tags init pipeline
//        ///////////////////////////
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam-2"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//        telemetry.setMsTransmissionInterval(50);
//
//
//
//
//        AprilTagDetection tagOfInterest = null;
//
//        while (!opModeIsActive()) {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            if(currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections) {
//                    if(tag.id == LEFT || tag.id == RIGHT || tag.id == MIDDLE) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        robotEndingPose = tag.id;
//                        break;
//                    }
//                }
//                if(tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//            else {
//                telemetry.addLine("Don't see tag of interest :(");
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//
//        waitForStart();
//    }
//
//
//
//
//    void tagToTelemetry(AprilTagDetection detection) {
//        Ending_Location = detection.id;
////        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
////        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
////        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
////        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
////        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
////        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
////        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//        telemetry.addData("Thing :", Ending_Location);
//        telemetry.update();
//    }
//}
