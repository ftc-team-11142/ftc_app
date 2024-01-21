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
//        DcMotorEx lift_left = hardwareMap.get(DcMotorEx.class, "lift left");
//        DcMotorEx lift_right = hardwareMap.get(DcMotorEx.class, "lift right");
//        DcMotorEx hanger_left = hardwareMap.get(DcMotorEx.class, "hanger left");
//        MotorEx hanger_right = new MotorEx(hardwareMap, "hanger right");

        // Servos
//        Servo air_plane_launcher = hardwareMap.get(Servo.class, "air plane launcher");
//        Servo arm_left = hardwareMap.get(Servo.class, "arm left");
//        Servo arm_right = hardwareMap.get(Servo.class, "arm right");
//        Servo pixel_holder = hardwareMap.get(Servo.class, "pixel holder");
//        Servo claw = hardwareMap.get(Servo.class, "claw");


        // Sensors
        BNO055IMU imu_sensor = hardwareMap.get(BNO055IMU.class, "imu");
//        DistanceSensor claw_sensor_left = hardwareMap.get(DistanceSensor.class, "claw sensor left");
//        DistanceSensor claw_sensor_right = hardwareMap.get(DistanceSensor.class, "claw sensor right");


        // Sub-Assemblies
        this.drivetrain = new Drivetrain(front_left, front_right.motorEx, back_left, back_right.motorEx, imu_sensor/*, pixel_holder*/);
//        this.intake = new Intake(claw, claw_sensor_left, claw_sensor_right);
//        this.lift = new Lift(lift_left, lift_right, arm_left, arm_right, air_plane_launcher);
//        this.hanger = new Hanger(hanger_left, hanger_right.motorEx);
//        this.odometry = new Odometry(front_right, hanger_right, back_right, front_left,back_left);
    }
}
