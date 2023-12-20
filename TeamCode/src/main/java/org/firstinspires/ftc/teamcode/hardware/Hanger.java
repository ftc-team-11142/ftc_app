package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Hanger {

    private final DcMotorEx hanger_left;
    private final DcMotorEx hanger_right;
    public Hanger (DcMotorEx hanger_left, DcMotorEx hanger_right) {
        this.hanger_left = hanger_left;
        this.hanger_right = hanger_right;

        hanger_right.setDirection(DcMotorSimple.Direction.REVERSE);

        hanger_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanger_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders() {
        hanger_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hanger_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double pow) {
        hanger_left.setPower(pow);
        hanger_right.setPower(pow);
    }
}
