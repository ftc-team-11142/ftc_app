package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Vision Test")
public class VisionTest extends LoggingOpMode {

    private WebcamName camera;
    private VisionPortal vision_portal;
    private TfodProcessor tfod_processor;
    private AprilTagProcessor apriltag_processor;

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        apriltag_processor = AprilTagProcessor.easyCreateWithDefaults();
        tfod_processor = TfodProcessor.easyCreateWithDefaults();
        vision_portal = VisionPortal.easyCreateWithDefaults(camera, apriltag_processor, tfod_processor);


    }

    @Override
    public void init_loop() {
        super.init_loop();
//        ArrayList<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> apriltag_detections;  // list of all detections
//        AprilTagDetection apriltag_detection;         // current detection in for() loop
//        double x = 0;
//        double y = 0;
//        double z = 0;
//
//        apriltag_detections = apriltag_processor.getDetections();
//
//// Cycle through through the list and process each AprilTag.
//        for (detection : apriltag_detections) {
//
//            if (detections.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
//                myAprilTagIdCode = myAprilTagDetection.id;
//
//                // Now take action based on this tag's ID code, or store info for later action.

//            }
//        }
//
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
