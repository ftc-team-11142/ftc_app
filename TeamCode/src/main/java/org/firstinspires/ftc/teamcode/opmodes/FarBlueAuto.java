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
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;
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

import java.lang.annotation.ElementType;
import java.util.ArrayList;

@Config
@Autonomous(name = "!! FAR Blue Auto !!")
public class FarBlueAuto extends LoggingOpMode{
    private Drivetrain drivetrain;
    private Lift lift;
    private Odometry odometry;
    private Arm arm;

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

    public static int x1 = 430;
    public static int y1 = 320;
    public static int x2 = 20;
    public static int y2 = 380;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);
        drivetrain = robot.drivetrain;
        lift = robot.lift;
        odometry = robot.odometry;
        arm = robot.arm;

        Pose2d start_pose = new Pose2d(0,0,new Rotation2d(Math.toRadians(0.1)));
        odometry.updatePose(start_pose);

        odometry.resetEncoders();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        color_pipeline = new ColorPipeline(370,240,10,240,"far blue");

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

        lift.resetEncoders();
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
        lift.resetEncoders();
        odometry.resetEncoders();
        drivetrain.resetEncoders();
        pixel_dragger.setPosition(0.508);
        camera.closeCameraDevice();
    }

    @Override
    public void loop() {

        drivetrain.updateHeading();

        odometry.updatePose(-drivetrain.getHeading());
        Pose2d odometryPose = odometry.getPose();

        if (result.equals("center")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(31, 3, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dragger.setPosition(0.334);
                        arm.setPosition(0);
                        timer.reset();
                    }
                    break;
                case 1:
                    if (timer.seconds() > 1.20) {
                        drivetrain.autoMove(20, 3, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 2:
                    drivetrain.autoMove(20, 25, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 1.2) {
                        main_id += 1;
                    }
                    break;
                case 3:
                    drivetrain.autoMove(46, 25, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    drivetrain.autoMove(46, 10, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(46, -71, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        timer.reset();
                    }
                    break;
                case 6:
                    drivetrain.autoMove(25, -88.9, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 2.2) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.67);
                        timer.reset();
                    }
                    break;
                case 7:
                    if (timer.seconds() > 3) {
                        drivetrain.autoMove(25, -83, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 8:
                    drivetrain.autoMove(46, -83, 270, 0.5, 0.5, 2, odometryPose, telemetry);
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
                    drivetrain.autoMove(21.5, 13, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        arm.setPosition(0);
                    }
                    break;
                case 1:
                    drivetrain.autoMove(20, 13, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        timer.reset();
                        pixel_dragger.setPosition(0.334);
                    }
                    break;
                case 2:
                    if (timer.seconds() > 2) {
                        drivetrain.autoMove(19, 24.5, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached() || timer.seconds() > 3) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 3:
                    drivetrain.autoMove(46, 24.5, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 1.3) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    drivetrain.autoMove(46, 10, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(46, -71, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 6:
                    drivetrain.autoMove(31, -88.9, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.67);
                        timer.reset();
                    }
                    break;
                case 7:
                    if (timer.seconds() > 3) {
                        drivetrain.autoMove(31, -83, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 8:
                    drivetrain.autoMove(46, -83, 270, 0.5, 0.5, 2, odometryPose, telemetry);
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
                    drivetrain.autoMove(28.25, 5.387, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        arm.setPosition(0);
                    }
                    break;
                case 1:
                    drivetrain.autoMove(26, 9.784, 270, 0.8, 0.8, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 2:
                    drivetrain.autoMove(26, -3.8, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        pixel_dragger.setPosition(0.334);
                        timer.reset();
                    }
                    break;
                case 3:
                    if (timer.seconds() > 1.2) {
                        drivetrain.autoMove(26, 10, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 4:
                    drivetrain.autoMove(46, 10, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 1.8) {
                        main_id += 1;
                    }
                    break;
                case 5:
                    drivetrain.autoMove(46, -71, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                        timer.reset();
                    }
                    break;
                case 6:
                    drivetrain.autoMove(22, -88.9, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached() || timer.seconds() > 2.2) {
                        main_id += 1;
                        pixel_dropper.setPosition(0.67);
                        timer.reset();
                    }
                    break;
                case 7:
                    if (timer.seconds() > 3) {
                        drivetrain.autoMove(22, -83, 270, 0.5, 0.5, 2, odometryPose, telemetry);
                        if (drivetrain.hasReached()) {
                            main_id += 1;
                            timer.reset();
                        }
                    }
                    break;
                case 8:
                    drivetrain.autoMove(46, -83, 270, 0.5, 0.5, 2, odometryPose, telemetry);
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

        arm.update();
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
        super.stop();
    }
}
