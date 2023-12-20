package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final DcMotorEx intake_spinner;

    public Intake(DcMotorEx intake_spinner) {
        this.intake_spinner = intake_spinner;

        intake_spinner.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setSpinnerPower(double pow) {
        intake_spinner.setPower(pow);
    }

}