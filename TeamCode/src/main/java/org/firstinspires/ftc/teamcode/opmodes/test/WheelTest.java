package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@Config
@TeleOp(name = "Wheel Test")
public class WheelTest extends LoggingOpMode {

    private DcMotorEx intake_spinner;
//    private DcMotorEx front_left, front_right, back_left, back_right;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);

//        front_left = hardwareMap.get(DcMotorEx.class, "front left");
//        front_right = hardwareMap.get(DcMotorEx.class, "front right");
//        back_left = hardwareMap.get(DcMotorEx.class, "back left");
//        back_right = hardwareMap.get(DcMotorEx.class, "back right");
        intake_spinner = hardwareMap.get(DcMotorEx.class, "u");

//        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

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
    }

    @Override
    public void loop() {

//        double forward_power = gamepad1.left_stick_y;
//        double strafe_power = -gamepad1.left_stick_x;
//        double turn_power = gamepad1.right_stick_x;
//
//        front_left.setPower(((forward_power + strafe_power + (turn_power + 0))));
//        front_right.setPower(((forward_power - strafe_power - (turn_power + 0))));
//        back_left.setPower(((forward_power - strafe_power + (turn_power + 0))));
//        back_right.setPower(((forward_power + strafe_power - (turn_power + 0))));

        intake_spinner.setPower(-gamepad2.left_stick_y);

        telemetry.addData("Intake Power", intake_spinner.getPower());
        telemetry.update();

        LoopTimer.resetTimer();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
