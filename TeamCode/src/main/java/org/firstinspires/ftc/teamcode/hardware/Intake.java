package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private final Servo claw_left;
    private final Servo claw_right;
    private final Servo claw_wrist;
    private final DistanceSensor claw_sensor_left;
    private final DistanceSensor claw_sensor_right;

    public Intake(Servo claw_left, Servo claw_right, Servo claw_wrist, DistanceSensor claw_sensor_left, DistanceSensor claw_sensor_right) {
        this.claw_left = claw_left;
        this.claw_right = claw_right;
        this.claw_wrist = claw_wrist;
        this.claw_sensor_left = claw_sensor_left;
        this.claw_sensor_right = claw_sensor_right;
    }

    public void setLeftPosition(double pos) {
        claw_left.setPosition(pos);
    }

    public double getLeftPosition() {
        return claw_left.getPosition();
    }

    public void setRightPosition(double pos) {
        claw_right.setPosition(pos);
    }

    public double getRightPosition() {
        return claw_right.getPosition();
    }

    public void setWristPosition(double pos) {
        claw_wrist.setPosition(pos);
    }

    public double getWristPosition() {
        return claw_wrist.getPosition();
    }

    public double getLeftDistance() {
        return claw_sensor_left.getDistance(DistanceUnit.MM);
    }

    public double getRightDistance() {
        return claw_sensor_right.getDistance(DistanceUnit.MM);
    }

}