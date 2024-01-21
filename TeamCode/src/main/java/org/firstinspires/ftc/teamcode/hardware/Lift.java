package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.tensorflow.lite.annotations.UsedByReflection;

public class Lift {

    private final DcMotorEx lift_left;
    private final DcMotorEx lift_right;
    private final Servo arm_left;
    private final Servo arm_right;

    private final Servo air_plane_launcher;

    public Lift(DcMotorEx lift_left, DcMotorEx lift_right, Servo arm_left, Servo arm_right, Servo air_plane_launcher) {
        this.lift_left = lift_left;
        this.lift_right = lift_right;
        this.arm_left = arm_left;
        this.arm_right = arm_right;
        this.air_plane_launcher = air_plane_launcher;
        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders() {
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getLiftPosition() {
        return lift_left.getCurrentPosition();
    }

    public void setPower(double pow) {
        lift_left.setPower(pow);
        lift_right.setPower(-pow);
    }

    public void setArmPosition(double pos) {
        arm_left.setPosition(pos);
        arm_right.setPosition(pos); //TODO
    }

//    public void setArmPower(double pos) {
//        arm_left.setPosition(pos);
//        arm_right.setPosition(pos);
//    }

    public void setAPLPosition(double pos) {
        air_plane_launcher.setPosition(pos);
    }
}
