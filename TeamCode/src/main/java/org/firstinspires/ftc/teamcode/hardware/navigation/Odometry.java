package org.firstinspires.ftc.teamcode.hardware.navigation;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Odometry {

    private final MotorEx front_right;
    private final MotorEx hanger_right;
    private final MotorEx back_right;
    private final DcMotorEx front_left;
    private final DcMotorEx back_left;

    private final HolonomicIMUOdometry odometry;

    private final Encoder left_odometer;
    private final Encoder right_odometer;
    private final Encoder center_odometer;

    private final double TRACKWIDTH = 9.5;
    private final double CENTER_WHEEL_OFFSET = 1.25;
    private final double WHEEL_DIAMETER = 1.88976;
    private final double TICKS_PER_REV = 2000;
    private final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public Odometry(MotorEx front_right, MotorEx hanger_right, MotorEx back_right, DcMotorEx front_left, DcMotorEx back_left) {
        this.front_right = front_right;
        this.hanger_right = hanger_right;
        this.back_right = back_right;
        this.front_left = front_left;
        this.back_left = back_left;

        left_odometer = front_right.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        right_odometer = hanger_right.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        center_odometer = back_right.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        right_odometer.setDirection(MotorEx.Direction.REVERSE);

        odometry = new HolonomicIMUOdometry(
                left_odometer::getDistance,
                right_odometer::getDistance,
                center_odometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        resetEncoders();

        Pose2d start_pose = new Pose2d(0,0, new Rotation2d(Math.toRadians(0)));
        odometry.updatePose(start_pose);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void updatePose(double heading) {
        odometry.updatePose(heading);
    }

    public void updatePose(Pose2d pose) {
        odometry.updatePose(pose);
    }

    public Pose2d getPose() {
        return odometry.getPose();
    }

    public void resetEncoders() {
        left_odometer.reset();
        right_odometer.reset();
        center_odometer.reset();
    }

}
