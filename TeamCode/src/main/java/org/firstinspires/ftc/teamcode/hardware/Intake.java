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
//    private final DistanceSensor claw_sensor_left;
//    private final DistanceSensor claw_sensor_right;

    public Intake(Servo claw_left, Servo claw_right/*, DistanceSensor claw_sensor_left, DistanceSensor claw_sensor_right*/) {
        this.claw_left = claw_left;
        this.claw_right = claw_right;
//        this.claw_sensor_left = claw_sensor_left;
//        this.claw_sensor_right = claw_sensor_right;
    }

    public void setClawleftPosition(double pos) {
        claw_left.setPosition(pos);
    }

    public void setClawRightPosition(double pos) {
        claw_right.setPosition(pos);
    }

//    public double getLeftDistance() {
//        return claw_sensor_left.getDistance(DistanceUnit.MM);
//    }
//
//    public double getRightDistance() {
//        return claw_sensor_right.getDistance(DistanceUnit.MM);
//    }

}