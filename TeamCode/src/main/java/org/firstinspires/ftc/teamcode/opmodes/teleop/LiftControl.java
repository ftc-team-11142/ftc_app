package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

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

    public LiftControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        ax_lift = controllerMap.getAxisMap("lift:axis", "gamepad2", "right_stick_y");
        lift_high_button = controllerMap.getButtonMap("lift:high", "gamepad2", "y");
        lift_mid_button = controllerMap.getButtonMap("lift:mid", "gamepad2", "b");
        lift_down_button = controllerMap.getButtonMap("lift:down", "gamepad2", "a");
    }

    @Override
    public void update(Telemetry telemetry) {

        double lift_postion_sum = lift.getLeftPosition() + ax_lift.get();
        lift.setPosition(lift_postion_sum);

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
            lift.setPosition(0);
        }

        if (lift_mid) {
            lift.setPosition(0);
        }

        if (lift_down) {
            if (sequencing_timer.seconds() > 1.8) {
                lift.setPosition(0);
            }
        }

        telemetry.addData("Arm Left", lift.getLeftPosition());
        telemetry.addData("Arm Right", lift.getRightPosition());
        telemetry.update();

    }
}
