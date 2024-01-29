package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class RedRobotControl extends ControlModule{

    private Lift lift;
    private Intake intake;
    private Arm arm;
    private Drivetrain drivetrain;

    private ControllerMap.ButtonEntry intake_button;
    private ControllerMap.ButtonEntry high_button;
    private ControllerMap.ButtonEntry mid_button;
    private ControllerMap.ButtonEntry down_button;

    private ControllerMap.AxisEntry ax_drive_strafe;
    private ControllerMap.AxisEntry ax_drive_forward;
    private ControllerMap.AxisEntry ax_drive_rotate;
    private ControllerMap.AxisEntry ax_slow;

    private ControllerMap.ButtonEntry reverse_drive_button;
    private ControllerMap.ButtonEntry backdrop_lock_button;
    private ControllerMap.ButtonEntry field_centric_button;
    private ControllerMap.ButtonEntry angled_turn_button;

    private ControllerMap.ButtonEntry turn_0_button;
    private ControllerMap.ButtonEntry turn_90_button;
    private ControllerMap.ButtonEntry turn_180_button;
    private ControllerMap.ButtonEntry turn_270_button;

    private ElapsedTime sequencing_timer = new ElapsedTime();

    private String state = "down";
    private String previous_state = state;

    private boolean claw_open = true;

    public static double down_lift_delay = 0.8;
    public static double up_arm_wrist_delay = 0.8;

    public static double lift_down_position = 0;
    public static double arm_down_position = 0;
    public static double wrist_down_position = 0;

    public static double lift_mid_position = 0;
    public static double arm_mid_position = 0;
    public static double wrist_mid_position = 0;

    public static double lift_high_position = 0;
    public static double arm_high_position = 0;
    public static double wrist_high_position = 0;

    public static double forward_speed = 1;
    public static double strafe_speed = 1.1;
    public static double turn_speed = 0.75;

    private double lift_target = 0;
    private double lift_power = 0;

    private boolean field_centric = false;
    private boolean backdrop_lock = false;
    private boolean reverse_drive = false;

    private double heading_delta = 0;
    private double heading_was = 0;
    private double heading_unwrapped = 0;
    private double wraparounds = 0;

    private double speed_dependent_steering = 0.5;

    private double backdrop_lock_angle = 270;
    private double turn_angle = 0;
    private boolean angled_turn = false;

    public static double turn_kp = 0.007;
    public static double turn_ki = 0.125;
    public static double turn_kd = 0.0028;
    public static double turn_a = 0.8;
    public static double turn_max_i_sum = 1;
    public static double turn_clip = 1;

    private final PID turn_pid = new PID(turn_kp,turn_ki,turn_kd,0.2,turn_max_i_sum,turn_a);


    public RedRobotControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        this.intake = robot.intake;
        this.arm = robot.arm;
        this.drivetrain = robot.drivetrain;

        intake_button = controllerMap.getButtonMap("intake:claw", "gamepad2","x");
        high_button = controllerMap.getButtonMap("main_mechanism:high", "gamepad2", "y");
        mid_button = controllerMap.getButtonMap("main_mechanism:mid", "gamepad2", "b");
        down_button = controllerMap.getButtonMap("main_mechanism:down", "gamepad2", "a");

        ax_drive_strafe = controllerMap.getAxisMap("drive:stafe_axis", "gamepad1", "left_stick_x");
        ax_drive_forward = controllerMap.getAxisMap("drive:forward_axis", "gamepad1", "left_stick_y");
        ax_drive_rotate = controllerMap.getAxisMap("drive:rotate_axis", "gamepad1", "right_stick_x");
        ax_slow = controllerMap.getAxisMap("drive:slow", "gamepad1", "left_trigger");

        reverse_drive_button = controllerMap.getButtonMap("drive:reverse_drive_button", "gamepad1","b");
        backdrop_lock_button = controllerMap.getButtonMap("drive:backdrop_lock_button", "gamepad1","y");
        angled_turn_button = controllerMap.getButtonMap("drive:angled_turn_button", "gamepad1","x");
        field_centric_button = controllerMap.getButtonMap("drive:field_centric_button", "gamepad1","a");

        turn_0_button = controllerMap.getButtonMap("drive:turn_0_button", "gamepad1","dpad_up");
        turn_90_button = controllerMap.getButtonMap("drive:turn_90_button", "gamepad1","dpad_left");
        turn_180_button = controllerMap.getButtonMap("drive:turn_180_button", "gamepad1","dpad_down");
        turn_270_button = controllerMap.getButtonMap("drive:turn_270_button", "gamepad1","dpad_right");
    }

    @Override
    public void update(Telemetry telemetry) {

        //heading
        drivetrain.updateHeading();
        double heading = drivetrain.getHeading();

        //angled turn
        if(turn_0_button.edge() == -1) {
            turn_angle = 0;
            angled_turn = true;
        }
        if(turn_90_button.edge() == -1) {
            turn_angle = 90;
            angled_turn = true;
        }
        if(turn_180_button.edge() == -1) {
            turn_angle = 180;
            angled_turn = true;
        }
        if(turn_270_button.edge() == -1) {
            turn_angle = 270;
            angled_turn = true;
        }

        //stopping angled turn
        if (Math.abs(ax_drive_rotate.get()) > 0.05) {
            angled_turn = false;
        }

        //claw
        if (intake_button.edge() == -1) {
            claw_open = !claw_open;
        }

        if (claw_open) {
            intake.setLeftPosition(0.2);
            intake.setRightPosition(0.80);
        }
        else {
            intake.setLeftPosition(0);
            intake.setRightPosition(1);
        }

        //sequencing states
        switch (state) {
            case "down":
                if (previous_state.equals("mid") || previous_state.equals("high")) {
                    if (sequencing_timer.seconds() > down_lift_delay) {
                        lift_target = lift_down_position;
                    }
                }
                else {
                    lift_target = lift_down_position;
                }

                arm.setPosition(arm_down_position);
                intake.setWristPosition(wrist_down_position);
                break;
            case "mid":
                if (previous_state.equals("down")) {
                    if (sequencing_timer.seconds() > up_arm_wrist_delay) {
                        arm.setPosition(arm_mid_position);
                        intake.setWristPosition(wrist_mid_position);
                    }
                }
                else {
                    arm.setPosition(arm_mid_position);
                    intake.setWristPosition(wrist_mid_position);
                }

                lift_target = lift_mid_position;
                break;
            case "high":
                if (previous_state.equals("down")) {
                    if (sequencing_timer.seconds() > up_arm_wrist_delay) {
                        arm.setPosition(arm_high_position);
                        intake.setWristPosition(wrist_high_position);
                    }
                }
                else {
                    arm.setPosition(arm_high_position);
                    intake.setWristPosition(wrist_high_position);
                }

                lift_target = lift_high_position;
                break;
        }

        //lift
        lift_power = (lift_target - lift.getPosition()) * 0.02;
        if (Math.abs(lift_target - lift.getPosition()) > 20) {
            lift.setPower(lift_power);
        }
        else {
            lift.setPower(0);
        }

        //setting states
        if (down_button.get()) {
            previous_state = state;
            state = "down";
            sequencing_timer.reset();
        }
        if (mid_button.get()) {
            previous_state = state;
            state = "mid";
            sequencing_timer.reset();
        }
        if (high_button.get()) {
            previous_state = state;
            state = "high";
            sequencing_timer.reset();
        }

        //drivetrain power
        double slow = 1 - (ax_slow.get() / 3);

        double y = -ax_drive_forward.get() * forward_speed * slow;
        double x = ax_drive_strafe.get() * strafe_speed * slow;
        double rx = ax_drive_rotate.get() * turn_speed * slow;

        //this makes turning slower as lateral motion gets faster
        rx *= (1 - (Math.sqrt(Math.pow(ax_drive_forward.get(), 2) + Math.pow(ax_drive_strafe.get() * 0.8, 2)) * speed_dependent_steering)); //pythagorean theorem

        //drive modes
        if(reverse_drive_button.edge() == -1) {
            reverse_drive = !reverse_drive;
        }
        if(backdrop_lock_button.edge() == -1) {
            backdrop_lock = !backdrop_lock;
        }
        if(field_centric_button.edge() == -1) {
            field_centric = !field_centric;
        }
        if(angled_turn_button.edge() == -1) {
            angled_turn = !angled_turn;
        }

        //angled turn
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

        //drift correction
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

        //drivetrain
        double heading_radians = Math.toRadians(heading);

        double rotX = x * Math.cos(-heading_radians) - y * Math.sin(-heading_radians);
        double rotY = x * Math.sin(-heading_radians) + y * Math.cos(-heading_radians);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        if (!reverse_drive) {
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

        telemetry.addData("Backdrop Lock", backdrop_lock);
        telemetry.addData("Field Centric", field_centric);
        telemetry.addData("Reverse", reverse_drive);
        telemetry.addData("State", state);
        telemetry.addData("Previous State", previous_state);
        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Current", lift.getPosition());
        telemetry.addData("Slow", slow);
        telemetry.update();
    }
}
