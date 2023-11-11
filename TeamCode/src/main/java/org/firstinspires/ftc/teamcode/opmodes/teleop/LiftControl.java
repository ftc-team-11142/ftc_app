package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class LiftControl extends ControlModule{

    private Lift lift;
    private ControllerMap.AxisEntry ax_lift_right_y;
    private ControllerMap.ButtonEntry dpad_left;
    private ControllerMap.ButtonEntry dpad_right;
    private ControllerMap.ButtonEntry dpad_up;
    private ControllerMap.ButtonEntry dpad_down;

    private double rot_pos = 0;
    private double apl_pos = 0;

    public LiftControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        ax_lift_right_y = controllerMap.getAxisMap("lift:right_y", "gamepad2", "right_stick_y");

        dpad_left = controllerMap.getButtonMap("intake:dpad_left", "gamepad1","dpad_left");
        dpad_right = controllerMap.getButtonMap("intake:dpad_right", "gamepad1","dpad_right");
        dpad_up = controllerMap.getButtonMap("intake:dpad_up", "gamepad1","dpad_up");
        dpad_down = controllerMap.getButtonMap("intake:dpad_down", "gamepad1","dpad_down");
    }

    @Override
    public void update(Telemetry telemetry) {

        lift.setPower(ax_lift_right_y.get());

        if (dpad_left.get()) {
            rot_pos -= 0.1;
        }
        if (dpad_right.get()) {
            rot_pos += 0.1;
        }

        if (dpad_down.get()) {
            apl_pos -= 0.1;
        }
        if (dpad_up.get()) {
            apl_pos += 0.1;
        }

        lift.setAPLPosition(apl_pos);
        lift.setTrayPosition(rot_pos);
    }
}
