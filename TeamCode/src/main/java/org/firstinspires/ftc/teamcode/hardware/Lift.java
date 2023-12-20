package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.tensorflow.lite.annotations.UsedByReflection;

public class Lift {

    private final DcMotorEx lift;
    private final Servo lift_servo;
    private final CRServo air_plane_launcher;

    public Lift(DcMotorEx lift, Servo lift_servo, CRServo air_plane_launcher) {
        this.lift = lift;
        this.lift_servo = lift_servo;
        this.air_plane_launcher = air_plane_launcher;
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getLiftPosition() {
        return lift.getCurrentPosition();
    }

    public void runToPosition(int pos) {
        lift.setTargetPosition(-1170);
    }

    public void setPower(double pow) {
        lift.setPower(pow);
    }

    public void setTrayPosition(double pos) {
        lift_servo.setPosition(pos);
    }

    public double getTrayPosition() {
        return lift_servo.getPosition();
    }

    public void setAPLPosition(double pos) {
        air_plane_launcher.setPower(pos);
    }

//    public double getAPLPosition() {
//        return air_plane_launcher.getPosition();
//    }
}
