package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private Servo lift_servo;

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift_servo = hardwareMap.get(Servo.class, "lift servo");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

//        lift.setPower((-1170 - lift.getCurrentPosition()) * 0.05);
        lift.setPower(gamepad1.left_stick_y);

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
