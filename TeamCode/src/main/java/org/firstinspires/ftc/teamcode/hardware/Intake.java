package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final CRServo intake_spinner;
    private final Servo intake_rotator;

    public Intake(CRServo intake_spinner, Servo intake_rotator) {
        this.intake_spinner = intake_spinner;
        this.intake_rotator = intake_rotator;
    }

    public void setSpinnerPower(double pow) {
        intake_spinner.setPower(pow);
    }

    public void setRotatorPosition(double pos) {
        intake_rotator.setPosition(pos);
    }

    public double getRotatorPosition() {
        return intake_rotator.getPosition();
    }

}