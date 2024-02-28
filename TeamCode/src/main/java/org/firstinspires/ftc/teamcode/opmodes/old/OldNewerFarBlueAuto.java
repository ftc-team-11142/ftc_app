package org.firstinspires.ftc.teamcode.opmodes.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.Odometry;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Config
@Autonomous(name = "!! Old Newer FAR Blue Auto !!")
public class OldNewerFarBlueAuto extends LoggingOpMode {
    private Drivetrain drivetrain;
//    private Lift lift;
    private Odometry odometry;
//    private Arm arm;

    private boolean stop = false;

    private String result = "Nothing";

    private int main_id = 0;

    private double lift_target = 0;
    private double lift_power = 0;

    private OpenCvCamera camera;
    private ColorPipeline color_pipeline;

    private Servo pixel_dragger;
    private Servo pixel_dropper;

    private FtcDashboard dashboard;

    private final Logger log = new Logger("Far Blue Auto");

    public static int x1 = 370;
    public static int y1 = 240;
    public static int x2 = 10;
    public static int y2 = 240;

    private boolean one_tag = false;
    private boolean two_tag = false;
    private boolean three_tag = false;

    private ArrayList<AprilTagDetection> currentDetections;

    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    private double tagsize = 0.166;

    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);
        drivetrain = robot.drivetrain;
//        lift = robot.lift;
        odometry = robot.odometry;
//        arm = robot.arm;

        Pose2d start_pose = new Pose2d(0,0,new Rotation2d(Math.toRadians(0.1)));
        odometry.updatePose(start_pose);

        odometry.resetEncoders();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        color_pipeline = new ColorPipeline(370,250,10,240,"far blue");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(color_pipeline);

        pixel_dragger = hardwareMap.get(Servo.class, "pixel dragger");
        pixel_dropper = hardwareMap.get(Servo.class, "pixel dropper");


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

//        lift.resetEncoders();
        odometry.resetEncoders();
        drivetrain.resetEncoders();

        result = color_pipeline.getLocation();

        telemetry.addData("Result", result);
//        telemetry.addData("color", color_pipeline.getColorValue());
        telemetry.addData("color", color_pipeline.getColorHSVValue());
        telemetry.addData("color 2", color_pipeline.getColorHSVValue2());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
//        lift.resetEncoders();
        odometry.resetEncoders();
        drivetrain.resetEncoders();
        pixel_dragger.setPosition(0.508);
        camera.setPipeline(aprilTagDetectionPipeline);
    }

    @Override
    public void loop() {

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

        drivetrain.updateHeading();

        odometry.updatePose(-drivetrain.getHeading());
        Pose2d odometryPose = odometry.getPose();

        if (result.equals("center")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(31, 3, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dragger.setPosition(0.334);
//                        arm.setPosition(0);
                        timer.reset();
                    }
                    break;
                case 1:
                    if (timer.seconds() > 0.7) {
                        drivetrain.autoMove(20, 3, 0, 1, 1, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 2:
                    drivetrain.autoMove(20, 25, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 1.2) {
                        main_id += 1;
                    }
                    break;
                case 3:
                    drivetrain.autoMove(46, 25, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    drivetrain.autoMove(46, 10, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(46, -71, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;

                    }
                    break;
                case 6:
                    drivetrain.autoMove(40, -71, 236.5, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() && one_tag && two_tag && three_tag) {
                        main_id += 1;
                        timer.reset();
                    }
                    break;
                case 7:
                    drivetrain.autoMove(25, -89, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 2.2) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.7);
                        timer.reset();
                    }
                    break;
                case 8:
                    if (timer.seconds() > 3) {
                        drivetrain.autoMove(25, -83, 270, 1, 1, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 9:
                    drivetrain.autoMove(46, -83, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 2.2) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.03);
                    }
                    break;
            }
        }
        if (result.equals("right")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(21.5, 13, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
//                        arm.setPosition(0);
                    }
                    break;
                case 1:
                    drivetrain.autoMove(19, 13, 0, 0.8, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        timer.reset();
                        pixel_dragger.setPosition(0.334);
                    }
                    break;
                case 2:
                    if (timer.seconds() > 0.7) {
                        drivetrain.autoMove(17, 24.5, 0, 1, 2, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached() || timer.seconds() > 3) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 3:
                    drivetrain.autoMove(46, 24.5, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 1.3) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    drivetrain.autoMove(46, 10, 270, 2, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(46, -71, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 6:
                    drivetrain.autoMove(31, -89, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.7);
                        timer.reset();
                    }
                    break;
                case 7:
                    if (timer.seconds() > 3) {
                        drivetrain.autoMove(31, -82, 270, 1, 1, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 8:
                    drivetrain.autoMove(46, -83, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 2.2) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.03);
                    }
                    break;
            }
        }
        if (result.equals("left")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(28.25, 5.387, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
//                        arm.setPosition(0);
                    }
                    break;
                case 1:
                    drivetrain.autoMove(26, 9.784, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 2:
                    drivetrain.autoMove(26, -3.8, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dragger.setPosition(0.334);
                        timer.reset();
                    }
                    break;
                case 3:
                    if (timer.seconds() > 0.7) {
                        drivetrain.autoMove(26, 10, 270, 1, 1, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 4:
                    drivetrain.autoMove(46, 10, 270, 2, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 1.8) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(46, -71, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        timer.reset();
                    }
                    break;
                case 6:
                    drivetrain.autoMove(22, -89, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 2.2) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.7);
                        timer.reset();
                    }
                    break;
                case 7:
                    if (timer.seconds() > 3) {
                        drivetrain.autoMove(22, -82, 270, 1, 1, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 8:
                    drivetrain.autoMove(46, -83, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 2.2) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.03);
                    }
                    break;
            }
        }

//        lift_power = (lift_target - lift.getLiftPosition()) * 0.03;
//        if (Math.abs(lift_target - lift.getLiftPosition()) > 20) {
//            lift.setPower(lift_power);
//        }
//        else {
//            lift.setPower(0);
//        }

//        arm.update();
        drivetrain.update(odometryPose, telemetry,false, main_id, false, false,0);

//        telemetry.addData("Lift Position", lift.getLiftPosition());
        telemetry.addData("One",one_tag);
        telemetry.addData("Two",two_tag);
        telemetry.addData("Three",three_tag);
        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Main ID", main_id);
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();

    }

    @Override
    public void stop() {
        super.stop();
        camera.closeCameraDevice();
    }
}
