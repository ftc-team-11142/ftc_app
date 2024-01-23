package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private final Servo arm_left;
    private final Servo arm_right;

    public Arm (Servo arm_left, Servo arm_right) {
        this.arm_left = arm_left;
        this.arm_right = arm_right;
    }

    public void setPosition(double pos) {
        arm_left.setPosition(pos);
        arm_right.setPosition(pos);
    }

    public double getLeftPosition() {
        return arm_left.getPosition();
    }

    public double getRightPosition() {
        return arm_right.getPosition();
    }

}
