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
    private final MotorEx back_right;
    private final MotorEx front_left;
    private final MotorEx back_left;
    private final MotorEx lift_left;

    private final HolonomicIMUOdometry odometry;

    private final Encoder left_odometer;
    private final Encoder right_odometer;
    private final Encoder center_odometer;

    private final double TRACKWIDTH = 15.5;
    private final double CENTER_WHEEL_OFFSET = 0.5;
    private final double WHEEL_DIAMETER = 1.88976;
    private final double TICKS_PER_REV = 2000;
    private final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public Odometry(MotorEx front_right, MotorEx back_right, MotorEx front_left, MotorEx back_left, MotorEx lift_left) {
        this.front_right = front_right;
        this.back_right = back_right;
        this.front_left = front_left;
        this.back_left = back_left;
        this.lift_left = lift_left;

        left_odometer = front_left.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        right_odometer = back_right.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        center_odometer = front_right.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);


        left_odometer.setDirection(MotorEx.Direction.REVERSE);
//        right_odometer.setDirection(MotorEx.Direction.REVERSE);
//        center_odometer.setDirection(MotorEx.Direction.REVERSE);

        odometry = new HolonomicIMUOdometry(
                left_odometer::getDistance,
                right_odometer::getDistance,
                center_odometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        resetEncoders();

        Pose2d start_pose = new Pose2d(0,0, new Rotation2d(Math.toRadians(0)));
        odometry.updatePose(start_pose);

        front_left.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_left.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
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
