package org.firstinspires.ftc.teamcode.robot.auto;//package org.firstinspires.ftc.teamcode.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auto.colorcv.ConeDeterminationPipeline;
//import org.firstinspires.ftc.teamcode.auto.conealign.Webcam1Pipeline;
//import org.firstinspires.ftc.teamcode.auto.conealign.Webcam2Pipeline;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous (name = "Auto Control testing ")
//public class controlAuto extends LinearOpMode {
//    OpenCvInternalCamera phoneCam;
//    ConeDeterminationPipeline.SkystoneDeterminationPipeline pipeline;
//    Webcam1Pipeline webcam1Pipeline;
//    Webcam2Pipeline webcam2Pipeline;
//
//    private BrainSTEMRobot robot;
//    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam-1"), cameraMonitorViewId);
//        pipeline = new ConeDeterminationPipeline.SkystoneDeterminationPipeline();
//
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });
//
//        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam-2"), cameraMonitorViewId2);
//        pipeline = new ConeDeterminationPipeline.SkystoneDeterminationPipeline();
//
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });
//
//        waitForStart();
//
//
//    }
//}
