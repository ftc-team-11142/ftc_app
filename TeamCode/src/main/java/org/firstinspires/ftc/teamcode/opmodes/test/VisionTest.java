package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;
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

    private OpenCvCamera camera;
    private ArrayList<AprilTagDetection> currentDetections;

    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    private double tagsize = 0.166;

    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private boolean one_tag = false;
    private boolean two_tag = false;
    private boolean three_tag = false;

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

    }

    @Override
    public void init_loop() {
        super.init_loop();
        double x = 0;
        double y = 0;
        double z = 0;

        currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections){
                if (tag.id == 1) {
                    one_tag = true;
                    break;
                }
                else {
                    one_tag = false;
                }
            }

            for (AprilTagDetection tag : currentDetections){
                if (tag.id == 2) {
                    two_tag = true;
                    x = tag.pose.x;
                    y = tag.pose.y;
                    z = tag.pose.z;
                    break;
                }
                else {
                    two_tag = false;
                }
            }

            for (AprilTagDetection tag : currentDetections){
                if (tag.id == 3) {
                    three_tag = true;
                    break;
                }
                else {
                    three_tag = false;
                }
            }
        }


        telemetry.addData("One",one_tag);
        telemetry.addData("Two",two_tag);
        telemetry.addData("Three",three_tag);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("z",z);
        telemetry.update();

    }

    @Override
    public void start() {
        super.start();
        camera.closeCameraDevice();
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
