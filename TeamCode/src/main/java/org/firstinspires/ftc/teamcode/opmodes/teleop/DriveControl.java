package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.Odometry;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class DriveControl extends ControlModule {

    private Drivetrain drivetrain;
//    private Odometry odometry;

    private ControllerMap.AxisEntry ax_drive_strafe;
    private ControllerMap.AxisEntry ax_drive_forward;
    private ControllerMap.AxisEntry ax_drive_rotate;
//    private ControllerMap.AxisEntry ax_horizontal_left_x;
    private ControllerMap.AxisEntry ax_slow;
    private ControllerMap.ButtonEntry reverse_drive;
    private ControllerMap.ButtonEntry backboard_lock;
    private ControllerMap.ButtonEntry dpad_up;
    private ControllerMap.ButtonEntry dpad_down;
    private ControllerMap.ButtonEntry dpad_left;
    private ControllerMap.ButtonEntry dpad_right;
    private ControllerMap.ButtonEntry field_centric_button;

    private double forward_speed = 1;
    private double strafe_speed = 1.1;
    private double turn_speed = 0.75;

    private boolean field_centric = false;
    private boolean smooth = false;
    private boolean direction_flip = false;
    private boolean under_rigging = false;

    private double heading_delta = 0;
    private double heading_was = 0;
    private double heading_unwrapped = 0;
    private double wraparounds = 0;

    private double target_heading = 0;

    private double speed_dependent_steering = 0.5; //0 is no speed dependent steering, 1 is too much

    //private double heading_p = 0.009;

//    private double forward;
//    private double strafe;
//    private double turn;
//
    private boolean angled_turn = false;
    private double turn_angle = 0;

    public static double turn_kp = 0.01; //was 0.007
    public static double turn_ki = 0.125;
    public static double turn_kd = 0.0028;
    public static double turn_a = 0.8;
    public static double turn_max_i_sum = 1;
    public static double turn_clip = 1;

    private final PID turn_pid = new PID(turn_kp,turn_ki,turn_kd,0.2,turn_max_i_sum,turn_a);

    public DriveControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.drivetrain = robot.drivetrain;
//        this.odometry = robot.odometry;

        ax_drive_strafe = controllerMap.getAxisMap("drive:stafe_axis", "gamepad1", "left_stick_x");
        ax_drive_forward = controllerMap.getAxisMap("drive:forward_axis", "gamepad1", "left_stick_y");
        ax_drive_rotate = controllerMap.getAxisMap("drive:rotate_axis", "gamepad1", "right_stick_x");
        ax_slow = controllerMap.getAxisMap("drive:slow", "gamepad1", "left_trigger");

        reverse_drive = controllerMap.getButtonMap("drive:reverse_drive", "gamepad1","dpad_left");
        backboard_lock = controllerMap.getButtonMap("drive:backboard_lock", "gamepad1","left_bumper");

        field_centric_button = controllerMap.getButtonMap("drive:field_centric_button", "gamepad1","dpad_up");

    }

    @Override
    public void init_loop(Telemetry telemetry) {

    }

    @Override
    public void update(Telemetry telemetry) {

        drivetrain.updateHeading();

        double heading = drivetrain.getHeading();

        if(reverse_drive.edge() == -1) {
            direction_flip = !direction_flip;
        }


        if (Math.abs(ax_drive_rotate.get()) > 0.05) {
            angled_turn = false;
        }

        if (field_centric_button.edge() == -1) {
            field_centric = !field_centric;
        }

        heading_delta = heading - heading_was;

        if (heading_delta > 320) {
            heading_delta -= 360;
            wraparounds -= 1;
        }
        if (heading_delta < -320) {
            heading_delta += 360;
            wraparounds += 1;
        }

        heading_unwrapped = -(heading + (wraparounds * 360));

        if (ax_drive_rotate.get() != 0) {
            heading_delta = 0;
        }

//        if (ax_drive_rotate.get() == 0) {
//            target_heading += (heading_unwrapped - target_heading) * 0.5;
//        }

        double slow = 1 - (ax_slow.get() / 3);

        double y = -ax_drive_forward.get() * forward_speed * slow;
        double x = ax_drive_strafe.get() * strafe_speed * slow;
        double rx = ax_drive_rotate.get() * turn_speed * slow;

        //this makes turning slower as lateral motion gets faster
        rx *= (1 - (Math.sqrt(Math.pow(ax_drive_forward.get(), 2) + Math.pow(ax_drive_strafe.get() * 0.8, 2)) * speed_dependent_steering)); //pythagorean theorem

        double rot = 0.0;
        if(Math.signum(-heading) == -1) {
            rot = ((-heading) + 360);
        }
        else {
            rot = -heading;
        }

        rot %= 360;

        if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot-360))) {
            rot -= 360;
        }if(Math.signum(-heading) == -1) {
            rot = ((-heading) + 360);
        }
        else {
            rot = -heading;
        }

        rot %= 360;

        if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot-360))) {
            rot -= 360;
        }
        if (angled_turn) {
            rx = turn_pid.getOutPut(turn_angle, rot, 0);
        }
//        rx = t
//        target_heading += rx * 11;

        double heading_radians = Math.toRadians(heading);

        double rotX = x * Math.cos(-heading_radians) - y * Math.sin(-heading_radians);
        double rotY = x * Math.sin(-heading_radians) + y * Math.cos(-heading_radians);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        if (!direction_flip) {
            if (field_centric) {
                drivetrain.move(rotY, rotX, rx, (heading_delta * 0.001), denominator);
            } else {
                drivetrain.move(y, x, rx, (heading_delta * 0.001));
            }
        }
        else {
            if (field_centric) {
                drivetrain.reverseMove(rotY, rotX, rx, (heading_delta * 0.001), denominator);
            } else {
                drivetrain.reverseMove(y, x, rx, (heading_delta * 0.001));
            }
        }

        heading_was = heading;

//        telemetry.addData("IMU Radians", Math.toRadians(heading));
//        telemetry.addData("IMU", heading);
//
//        telemetry.addData("Target Heading", target_heading);
//        telemetry.addData("Unwrapped Heading", heading_unwrapped);
//
//        telemetry.addData("rotX", rotX);
//        telemetry.addData("rotY", rotY);
//        telemetry.addData("denominator", denominator);
//
//        telemetry.addData("Heading: ", heading);
////        telemetry.addData("Angular Velocity: ", drivetrain.getAngularVelocity());
//
        telemetry.addData("Field Centric",field_centric);


    }
    @Override
    public void stop() {
        super.stop();
        // drivetrain.closeIMU();
    }
}