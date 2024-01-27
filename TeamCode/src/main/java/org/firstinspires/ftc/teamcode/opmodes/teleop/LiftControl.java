package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

@Config
public class LiftControl extends ControlModule{

    private Lift lift;
    private ControllerMap.AxisEntry ax_lift;
    private ControllerMap.ButtonEntry lift_high_button;
    private ControllerMap.ButtonEntry lift_mid_button;
    private ControllerMap.ButtonEntry lift_down_button;

    private boolean lift_high = false;
    private boolean lift_mid = false;
    private boolean lift_down = true;

    private ElapsedTime sequencing_timer = new ElapsedTime();

    private String previous_state = "lift_down";

    private double lift_target = 0;
    private double lift_power = 0;

    public LiftControl(String name) {
        super(name);
    }

    public static double high_lift = 0;
    public static double mid_lift = 0;
    public static double down_lift = 0;
    public static double lift_down_delay = 1.8;

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        ax_lift = controllerMap.getAxisMap("lift:axis", "gamepad2", "right_stick_y");
        lift_high_button = controllerMap.getButtonMap("lift:high", "gamepad2", "y");
        lift_mid_button = controllerMap.getButtonMap("lift:mid", "gamepad2", "b");
        lift_down_button = controllerMap.getButtonMap("lift:down", "gamepad2", "a");

        lift.resetEncoders();
        sequencing_timer.reset();
    }

    @Override
    public void init_loop(Telemetry telemetry) {
        super.init_loop(telemetry);
    }

    @Override
    public void update(Telemetry telemetry) {


        lift_target -= ax_lift.get();

        if (lift_high_button.edge() == -1) {
            if (lift_mid) {
                previous_state = "lift_mid";
            }
            if (lift_down) {
                previous_state = "lift_down";
            }

            lift_high = true;
            lift_mid = false;
            lift_down = false;
            sequencing_timer.reset();
        }

        if (lift_mid_button.edge() == -1) {
            if (lift_high) {
                previous_state = "lift_high";
            }
            if (lift_down) {
                previous_state = "lift_down";
            }
            lift_high = false;
            lift_mid = true;
            lift_down = false;
            sequencing_timer.reset();
        }

        if (lift_down_button.edge() == -1) {
            if (lift_high) {
                previous_state = "lift_high";
            }
            if (lift_mid) {
                previous_state = "lift_mid";
            }

            lift_high = false;
            lift_mid = false;
            lift_down = true;
            sequencing_timer.reset();
        }

        if (lift_high) {
//            if ( TODO Sequencing
            lift_target = high_lift;
        }

        if (lift_mid) {
            lift_target = mid_lift;
        }

        if (lift_down) {
            if (sequencing_timer.seconds() > lift_down_delay) {
                lift_target = down_lift;
            }
        }

        lift_power = (lift_target - lift.getPosition()) * 0.03;
        if (Math.abs(lift_target - lift.getPosition()) > 20) {
            lift.setPower(lift_power);
        }
        else {
            lift.setPower(0);
        }

        telemetry.addData("Lift", lift.getPosition());
        telemetry.update();

    }
}
