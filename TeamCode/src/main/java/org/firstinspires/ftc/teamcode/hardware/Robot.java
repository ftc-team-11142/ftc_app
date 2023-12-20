package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.navigation.Odometry;
import org.firstinspires.ftc.teamcode.util.Scheduler;
import org.firstinspires.ftc.teamcode.util.event.EventBus;

public class Robot {

    public Drivetrain drivetrain;
    public Odometry odometry;
    public Intake intake;
    public Lift lift;
    public Hanger hanger;
    public IMU imu;

    public EventBus eventBus = new EventBus();
    public Scheduler scheduler = new Scheduler(eventBus);

    private static Robot instance;
    public static Robot initialize(HardwareMap hardwareMap)
    {
        instance = new Robot(hardwareMap);
        return instance;
    }

    public static void close()
    {
        instance = null;
    }
    public static Robot instance()
    {
        return instance;
    }

    public Robot(HardwareMap hardwareMap)
    {
        // Motors
        DcMotorEx front_left = hardwareMap.get(DcMotorEx.class, "front left");
        MotorEx front_right = new MotorEx(hardwareMap, "front right");
        DcMotorEx back_left = hardwareMap.get(DcMotorEx.class, "back left");
        MotorEx back_right = new MotorEx(hardwareMap, "back right");
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");
        DcMotorEx hanger_left = hardwareMap.get(DcMotorEx.class, "hanger left");
        MotorEx hanger_right = new MotorEx(hardwareMap, "hanger right");
        DcMotorEx intake_spinner = hardwareMap.get(DcMotorEx.class, "intake spinner");

        // Servos
        CRServo air_plane_launcher = hardwareMap.get(CRServo.class, "air plane launcher");
        Servo lift_servo = hardwareMap.get(Servo.class, "lift servo");

        // Sensors
        BNO055IMU imu_sensor = hardwareMap.get(BNO055IMU.class, "imu");

        // Sub-Assemblies
        this.drivetrain = new Drivetrain(front_left, front_right.motorEx, back_left, back_right.motorEx, imu_sensor);
        this.intake = new Intake(intake_spinner);
        this.lift = new Lift(lift, lift_servo ,air_plane_launcher);
        this.hanger = new Hanger(hanger_left, hanger_right.motorEx);
        this.odometry = new Odometry(front_right, back_right, hanger_right, front_left,back_left);
    }
}
