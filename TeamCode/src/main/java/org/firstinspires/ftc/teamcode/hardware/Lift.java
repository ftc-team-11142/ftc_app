package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.tensorflow.lite.annotations.UsedByReflection;

public class Lift {

    private final Servo lift_left;
    private final Servo lift_right;


    public Lift(Servo lift_left,Servo lift_right) {
        this.lift_left = lift_left;
        this.lift_right = lift_right;
    }

    public double getLeftPosition() {
        return lift_left.getPosition();
    }

    public double getRightPosition() {
        return lift_right.getPosition();
    }

    public void setPosition(double pos) {
        lift_left.setPosition(pos);
        lift_right.setPosition(pos);
    }

}
