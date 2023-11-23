package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Autonomous(name = "!! Blue Auto !!")
public class BlueAuto extends LoggingOpMode{
    private Drivetrain drivetrain;
    private Lift lift;
    private Odometry odometry;

    private String result = "Nothing";

    private int main_id = 0;

    private double lift_target = 0;
    private double lift_power = 0;

    private OpenCvCamera camera;
    private ColorPipeline pipeline;

    private final Logger log = new Logger("Auto");

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);
        drivetrain = robot.drivetrain;
        lift = robot.lift;
        odometry = robot.odometry;

        Pose2d start_pose = new Pose2d(0,0,new Rotation2d(Math.toRadians(0.1)));
        odometry.updatePose(start_pose);

        odometry.resetEncoders();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorPipeline(1800, 1100, 400, 100,800,1100,400,100, "blue");

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
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

        result = pipeline.getLocation();

        telemetry.addData("Result", result);
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        lift.resetEncoders();
        odometry.resetEncoders();
        drivetrain.resetEncoders();
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
                    drivetrain.autoMove(-31, 2, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 1:
                    drivetrain.autoMove(-20, 2, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 2:
                    lift_target = -1315;
                    drivetrain.autoMove(-25, 20, 90, 1, 2, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 3:
                    lift.setTrayPosition(0.23);
                    drivetrain.autoMove(-30.19, 44.29, 90, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    lift.setTrayPosition(0.493);
                    break;

            }
        }
        if (result.equals("left")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(-21.914, 14.067572, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 1:
                    drivetrain.autoMove(-13, 14.3, 0, 0.8, 0.8, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 2:
                    lift_target = -1315;
                    drivetrain.autoMove(-15, 20, 90, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 3:
                    lift.setTrayPosition(0.23);
                    drivetrain.autoMove(-23.2177, 44.29, 90, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    lift.setTrayPosition(0.493);
                    break;
            }
        }
        if (result.equals("right")) {
            switch (main_id) {
                case 0:
                    drivetrain.autoMove(-28.25, 5.387, 0, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 1:
                    drivetrain.autoMove(-32, 9.784, 90, 0.8, 0.8, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 2:
                    drivetrain.autoMove(-31.7, 0.215, 90, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 3:
                    lift_target = -1315;
                    lift.setTrayPosition(0.22);
                    drivetrain.autoMove(-36.942, 44.29, 90, 0.5, 0.5, 2, odometryPose, telemetry);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                    break;
                case 4:
                    lift.setTrayPosition(0.493);
                    break;
            }
        }


        lift_power = (lift_target - lift.getLiftPosition()) * 0.03;
        if (Math.abs(lift_target - lift.getLiftPosition()) > 20) {
            lift.setPower(lift_power);
        }
        else {
            lift.setPower(0);
        }
        drivetrain.update(odometryPose, telemetry,false, main_id, false, false,0);

        telemetry.addData("Lift Position", lift.getLiftPosition());
        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Main ID", main_id);
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();

    }
}
