package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.tensorflow.lite.annotations.UsedByReflection;

public class Lift {

    private final DcMotorEx lift;


    public Lift(DcMotorEx lift) {
        this.lift = lift;
    }

    public void resetEncoders() {
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition() {
        return lift.getCurrentPosition();
    }

    public void setPower(double pow) {
        lift.setPower(pow);
    }

}
