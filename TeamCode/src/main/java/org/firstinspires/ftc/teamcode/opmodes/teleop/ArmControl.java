package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class ArmControl extends ControlModule{

    private Arm arm;
    private ControllerMap.AxisEntry ax_arm;
    private ControllerMap.ButtonEntry arm_high_button;
    private ControllerMap.ButtonEntry arm_mid_button;
    private ControllerMap.ButtonEntry arm_down_button;

    private boolean arm_high = false;
    private boolean arm_mid = false;
    private boolean arm_down = true;

    private ElapsedTime sequencing_timer = new ElapsedTime();

    private String previous_state = "arm_down";


    public ArmControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.arm = robot.arm;
        ax_arm = controllerMap.getAxisMap("arm:axis", "gamepad2", "left_stick_y");
        arm_high_button = controllerMap.getButtonMap("arm:high", "gamepad2", "y");
        arm_mid_button = controllerMap.getButtonMap("arm:mid", "gamepad2", "b");
        arm_down_button = controllerMap.getButtonMap("arm:down", "gamepad2", "a");
    }

    @Override
    public void update(Telemetry telemetry) {

        double arm_postion_sum = arm.getLeftPosition() + ax_arm.get();
        arm.setPosition(arm_postion_sum);

        if (arm_high_button.edge() == -1) {
            if (arm_mid) {
                previous_state = "arm_mid";
            }
            if (arm_down) {
                previous_state = "arm_down";
            }

            arm_high = true;
            arm_mid = false;
            arm_down = false;
            sequencing_timer.reset();
        }

        if (arm_mid_button.edge() == -1) {
            if (arm_high) {
                previous_state = "arm_high";
            }
            if (arm_down) {
                previous_state = "arm_down";
            }

            arm_high = false;
            arm_mid = true;
            arm_down = false;
            sequencing_timer.reset();
        }

        if (arm_down_button.edge() == -1) {

            if (arm_high) {
                previous_state = "arm_high";
            }
            if (arm_mid) {
                previous_state = "arm_mid";
            }

            arm_high = false;
            arm_mid = false;
            arm_down = true;
            sequencing_timer.reset();
        }

        if (arm_high) {
            if (sequencing_timer.seconds() > 1.8 && previous_state.equals("arm_down")) {
                arm.setPosition(0);
            }
            else {
                arm.setPosition(0);
            }
        }

        if (arm_mid) {
            if (sequencing_timer.seconds() > 1.8 && previous_state.equals("arm_down")) {
                arm.setPosition(0);
            }
            else {
                arm.setPosition(0);
            }
        }

        if (arm_down) {
            arm.setPosition(0);
        }

    }
}
