//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
//import org.firstinspires.ftc.teamcode.hardware.Intake;
//import org.firstinspires.ftc.teamcode.hardware.Lift;
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.hardware.navigation.Odometry;
//import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.util.LoopTimer;
//import org.firstinspires.ftc.teamcode.vision.ColorPipeline;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Config
//@Autonomous(name = "!! Forward !!")
//public class Forward extends LoggingOpMode{
//    private Drivetrain drivetrain;
//    private Lift lift;
//    private Intake intake;
//    private ElapsedTime timer = new ElapsedTime();
//
//    private int main_id = 0;
//
//
//    private final Logger log = new Logger("Far Blue Auto");
//
//    @Override
//    public void init() {
//        super.init();
//        Robot robot = Robot.initialize(hardwareMap);
//        drivetrain = robot.drivetrain;
//        lift = robot.lift;
//        intake = robot.intake;
//    }
//
//    @Override
//    public void init_loop() {
//        super.init_loop();
//
//        lift.resetEncoders();
//        drivetrain.resetEncoders();
//    }
//
//    @Override
//    public void start() {
//        super.start();
//        drivetrain.resetEncoders();
//        timer.reset();
//    }
//
//    @Override
//    public void loop() {
//
//        switch (main_id) {
//            case 0:
//                drivetrain.move(0, 0, 0, 0);
//                if (timer.seconds() > 2.5) {
//                    main_id += 1;
//                    timer.reset();
//                }
//                break;
//            case 1:
//                drivetrain.move(0.7,0,0,0);
//                if (timer.seconds() > 5) {
//                    main_id += 1;
//                    timer.reset();
//                }
//                break;
//            case 2:
//                intake.setSpinnerPower(0.5);
//                drivetrain.move(-0.7,0,0,0);
//                if (timer.seconds() > 1) {
//                    main_id += 1;
//                    timer.reset();
//                    intake.setSpinnerPower(0);
//                }
//                break;
//            case 3:
//                drivetrain.stop();
//                break;
//        }
//
//        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
//        telemetry.update();
//
//        LoopTimer.resetTimer();
//
//    }
//}
