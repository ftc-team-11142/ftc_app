package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.io.Serializable;

@Config
@TeleOp(name = "Lift Test")
public class LiftTest extends LoggingOpMode {

//    private CRServo lift_left;
    private Servo arm_left;
    private Servo arm_right;
    private Servo claw_left;
    private Servo claw_right;
    private Servo claw_wrist;
    private DcMotorEx intake_spinner;
    private DcMotorEx lift;
    private DcMotorEx hanger_left;
    private DcMotorEx hanger_right;
    private double sum = 0;
    private double wrist_sum = 0;
    private double lift_power = 0;
    private double lift_target = 0;
    private FtcDashboard dashboard;

    public static double kp = 0.03;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);

        arm_left = hardwareMap.get(Servo.class, "arm left");
        arm_right = hardwareMap.get(Servo.class, "arm right");

        claw_left = hardwareMap.get(Servo.class, "claw left");
        claw_right = hardwareMap.get(Servo.class, "claw right");
        claw_wrist = hardwareMap.get(Servo.class, "claw wrist");
//        lift_servo = hardwareMap.get(Servo.class, "lift servo");
//        intake_spinner = hardwareMap.get(DcMotorEx.class, "intake spinner");
//        hanger_left = hardwareMap.get(DcMotorEx.class, "hanger left");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_left.setDirection(Servo.Direction.REVERSE);
//        arm_right.setDirection(Servo.Direction.FORWARD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
//        claw_wrist.setPosition(0.92);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw_wrist.setDirection(Servo.Direction.REVERSE);
        timer.reset();
    }

    @Override
    public void loop() {

        if (timer.seconds() > 8) {
            lift_target = 1300;
        }

        if (gamepad1.a) {
            claw_left.setPosition(0.2);
            claw_right.setPosition(0.8);
        }
        if (gamepad1.b) {
            claw_left.setPosition(0);
            claw_right.setPosition(1);
        }

        if(gamepad1.dpad_up) {
            wrist_sum += 0.001;
        }
        if(gamepad1.dpad_down) {
            wrist_sum -= 0.001;
        }

        claw_wrist.setPosition(wrist_sum);

        sum = sum - gamepad1.right_stick_y*0.001;
        arm_right.setPosition(sum);
        arm_left.setPosition(sum);

        lift.setPower(-gamepad1.left_stick_y);
//        lift_power = (lift_target - lift.getCurrentPosition()) * kp;
//        if (Math.abs(lift_target - lift.getCurrentPosition()) > 20) {
//            lift.setPower(lift_power);
//        }
//        else {
//            lift.setPower(0);
//        }

//        arm_right.setPosition(sum);
        telemetry.addData("Lift Power", lift.getPower());
        telemetry.addData("Lift Encoder", lift.getCurrentPosition());
        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("wrist Position", claw_wrist.getPosition());
        telemetry.addData("Arm Position", sum);
        telemetry.addData("Wrist Sum", wrist_sum);
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.addData("Sum ", sum);
        telemetry.update();

        LoopTimer.resetTimer();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
