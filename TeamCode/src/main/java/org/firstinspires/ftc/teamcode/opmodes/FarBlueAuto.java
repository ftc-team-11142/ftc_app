package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.Odometry;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.ColorPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "!! FAR Blue Auto !!")
public class FarBlueAuto extends LoggingOpMode{
    private Drivetrain drivetrain;
//    private Lift lift;
    private Odometry odometry;
//    private Arm arm;

    private boolean stop = false;
    private boolean backdrop_clear = false;

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

    public static int x1 = 255;
    public static int y1 = 40;
    public static int x2 = 635;
    public static int y2 = 25;

    public static double strafe_var = -90;
    public static double forward_var = 20;

    ElapsedTime timer = new ElapsedTime();

    private ArrayList<AprilTagDetection> currentDetections;

    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    private double tagsize = 0.166;

    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

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
        color_pipeline = new ColorPipeline(x1,y1,x2,y2,"far blue");
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(color_pipeline);
        camera.setPipeline(aprilTagDetectionPipeline);

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

        pixel_dragger.setPosition(0.699);
        pixel_dropper.setPosition(0.875);

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
//        pixel_dragger.setPosition(0.508);
    }

    @Override
    public void loop() {

        double x = 0;
        double y = 0;
        double z = 0;

        drivetrain.updateHeading();

        odometry.updatePose(-drivetrain.getHeading());
        Pose2d odometryPose = odometry.getPose();

        currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (result.equals("center")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(14.43, 15.36, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 1:
                    drivetrain.autoMove(27, 10, 270, 2, 2, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 2:
                    drivetrain.autoMove(29.05, 5.46, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dragger.setPosition(0.572);
                    }
                    break;
                case 3:
                    drivetrain.autoMove(40, 8, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    drivetrain.autoMove(45.36, 0, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(45.36, -74.3, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 6:
                    drivetrain.autoMove(45.36, -74.3, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 7:
                    if(currentDetections.size() != 0) {
                        for (AprilTagDetection tag : currentDetections){
                            if (tag.id == 2) {
                                x = tag.pose.x;
                                y = tag.pose.y;
                                z = tag.pose.z;
                                Pose2d cam_pose = new Pose2d(x,y,new Rotation2d(Math.toRadians(-drivetrain.getHeading())));
                                odometry.updatePose(cam_pose);
                                main_id += 1;
                                break;
                            }
                        }
                    }
                    break;
                case 8:
                    drivetrain.autoMove(5, 2, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        timer.reset();
                        pixel_dropper.setPosition(0.205);
                    }
                    break;
                case 9:
                    if (timer.seconds() > 3) {
                        pixel_dropper.setPosition(0.205);
                        main_id += 1;
                    }
                    break;
                case 10:
                    drivetrain.autoMove(5, 4, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        timer.reset();

                    }
                    break;
//                case 6:
//                    drivetrain.autoMove(24, -90, 270, 1, 1, 2, odometryPose, telemetry);
//                    if (drivetrain.hasReached()) {
//                        main_id += 1;
//                        pixel_dropper.setPosition(0.205);
//                        timer.reset();
//                    }
//                    break;
//                case 7:
//                    drivetrain.autoMove(24, -90, 270, 1, 1, 2, odometryPose, telemetry);
//                    if (timer.seconds() > 3) {
//                        main_id += 1;
//                    }
//                    break;
//                case 8:
//                    drivetrain.autoMove(24, -88, 270, 1, 1, 2, odometryPose, telemetry);
//                    if (drivetrain.hasReached() && timer.seconds() > 3) {
//                        main_id += 1;
//                        pixel_dropper.setPosition(0.205);
//                    }
//                    break;
            }
        }
        if (result.equals("right")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(14.43, 15.36, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 1:
                    drivetrain.autoMove(16, 17.72, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dragger.setPosition(0.572);
                    }
                    break;
                case 3:
                    drivetrain.autoMove(40, 20, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    drivetrain.autoMove(45.36, 0, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(45.36, -74.3, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 6:
                    drivetrain.autoMove(30, -90, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.205);
                        timer.reset();
                    }
                    break;
                case 7:
                    drivetrain.autoMove(30, -90, 270, 1, 1, 2, odometryPose, telemetry);
                    if (timer.seconds() > 3) {
                        main_id += 1;
                    }
                    break;
                case 8:
                    drivetrain.autoMove(30, -88, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.205);
                    }
                    break;
            }
        }
        if (result.equals("left")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(14.43, 15.36, 0, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 1:
                    drivetrain.autoMove(17, 10, 270, 2, 2, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 2:
                    drivetrain.autoMove(19.79, -4, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dragger.setPosition(0.572);
                    }
                    break;
                case 3:
                    drivetrain.autoMove(19.79, 3, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    drivetrain.autoMove(45.36, 0, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(45.36, -74.3, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 6:
                    drivetrain.autoMove(forward_var, strafe_var, 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.205);
                        timer.reset();
                    }
                    break;
                case 7:
                    drivetrain.autoMove(forward_var, strafe_var, 270, 1, 1, 2, odometryPose, telemetry);
                    if (timer.seconds() > 3) {
                        main_id += 1;
                    }
                    break;
                case 8:
                    drivetrain.autoMove(forward_var, (strafe_var+2), 270, 1, 1, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.205);
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
        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Main ID", main_id);
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();

    }

    @Override
    public void stop() {
        camera.closeCameraDevice();
        super.stop();
    }
}
