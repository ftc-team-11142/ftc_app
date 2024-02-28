package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;

@TeleOp(name="Test Hanging")
public class TestHanging extends LoggingOpMode {

    private Robot robot;
    private DcMotorEx hanger;

    @Override
    public void init() {
        Robot robot = Robot.initialize(hardwareMap);

        hanger = hardwareMap.get(DcMotorEx.class, "hanger");

    }

    @Override
    public void loop() {
        hanger.setPower(-gamepad1.left_stick_y);
    }
}
