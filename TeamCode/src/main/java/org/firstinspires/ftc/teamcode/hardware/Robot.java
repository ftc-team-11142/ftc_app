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
    public Arm arm;
    public Drone drone;
    public IMU imu;

    public EventBus eventBus = new EventBus();
    public Scheduler scheduler = new Scheduler(eventBus);

    private static Robot instance;
    public static Robot initialize(HardwareMap hardwareMap)
    {

        instance = new Robot(hardwareMap);
        return instance;
    }

    public HardwareMap hardwareMap;

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
        MotorEx front_left = new MotorEx(hardwareMap, "front left");
        MotorEx front_right = new MotorEx(hardwareMap, "front right");
        MotorEx back_left = new MotorEx(hardwareMap, "back left");
        MotorEx back_right = new MotorEx(hardwareMap, "back right");
//        DcMotorEx hanger_left = hardwareMap.get(DcMotorEx.class, "hanger left");
////        DcMotorEx hanger_right = hardwareMap.get(DcMotorEx.class, "hanger right");
        MotorEx lift_left = new MotorEx(hardwareMap, "lift left");
//
//        // Servos
////        Servo drone_launcher = hardwareMap.get(Servo.class, "drone launcher");
//        Servo arm_left = hardwareMap.get(Servo.class, "arm left");
//        Servo arm_right = hardwareMap.get(Servo.class, "arm right");
//        Servo claw_right = hardwareMap.get(Servo.class, "claw right");
//        Servo claw_left = hardwareMap.get(Servo.class, "claw left");
//        Servo claw_wrist = hardwareMap.get(Servo.class, "claw wrist");
//        Servo drone_launcher = hardwareMap.get(Servo.class, "drone launcher");
////
////
////        // Sensors
        BNO055IMU imu_sensor = hardwareMap.get(BNO055IMU.class, "imu");
//        DistanceSensor claw_sensor_left = hardwareMap.get(DistanceSensor.class, "claw sensor left");
//        DistanceSensor claw_sensor_right = hardwareMap.get(DistanceSensor.class, "claw sensor right");


        // Sub-Assemblies
        this.drivetrain = new Drivetrain(front_left.motorEx, front_right.motorEx, back_left.motorEx, back_right.motorEx, imu_sensor);
//        this.intake = new Intake(claw_left,claw_right, claw_wrist /*, claw_sensor_left, claw_sensor_right*/);
//        this.lift = new Lift(lift);
//        this.hanger = new Hanger(hanger_left, hanger_right);
//        this.arm = new Arm(arm_left,arm_right);
//        this.drone = new Drone(drone_launcher);
        this.odometry = new Odometry(front_right, back_right, front_left ,back_left, lift_left);
        this.hardwareMap = hardwareMap;
    }
}
