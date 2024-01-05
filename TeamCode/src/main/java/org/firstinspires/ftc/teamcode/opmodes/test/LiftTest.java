package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.io.Serializable;

@TeleOp(name = "Lift Test")
public class LiftTest extends LoggingOpMode {

    private DcMotorEx lift;
//    private Servo lift_servo;
    private DcMotorEx intake_spinner;
    private DcMotorEx hanger_left;
    private DcMotorEx hanger_right;

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
//        lift_servo = hardwareMap.get(Servo.class, "lift servo");
//        intake_spinner = hardwareMap.get(DcMotorEx.class, "intake spinner");
//        hanger_left = hardwareMap.get(DcMotorEx.class, "hanger left");
//        hanger_right = hardwareMap.get(DcMotorEx.class, "hanger right");

//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

        lift.setPower(gamepad1.left_stick_y);
//        intake_spinner.setPower(-gamepad1.right_trigger);
//        hanger_left.setPower(-gamepad1.left_stick_y);
//        hanger_right.setPower(gamepad1.left_stick_y);

        telemetry.addData("Lift Encoder", lift.getCurrentPosition());
//        telemetry.addData("Lift Power", (-1170 - lift.getCurrentPosition()) * 0.00005);
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
