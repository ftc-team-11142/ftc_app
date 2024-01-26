package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.io.Serializable;

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
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        claw_wrist.setPosition(0.92);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            claw_left.setPosition(0.2);
            claw_right.setPosition(0.8);
        }
        if (gamepad1.b) {
            claw_left.setPosition(0);
            claw_right.setPosition(1);
        }

        if(gamepad1.dpad_up) {
            claw_wrist.setPosition(claw_wrist.getPosition()+0.001);
        }
        if(gamepad1.dpad_up) {
            claw_wrist.setPosition(claw_wrist.getPosition()-0.001);
        }

        sum = sum - gamepad1.right_stick_y*0.001;
        lift.setPower(-gamepad1.left_stick_y);
        arm_right.setPosition(sum);
        arm_left.setPosition(sum);
//        arm_right.setPosition(sum);
        telemetry.addData("Lift Power", lift.getPower());
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("wrist Position", claw_wrist.getPosition());
        telemetry.addData("Arm Position", sum);
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
