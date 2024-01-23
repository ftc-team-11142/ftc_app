package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Hanger;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class HangerControl extends ControlModule{

    private Hanger hanger;
    private ControllerMap.AxisEntry ax_hanger_down;
    private ControllerMap.AxisEntry ax_hanger_up;

    public HangerControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.hanger = robot.hanger;
        ax_hanger_up = controllerMap.getAxisMap("hanger:axis_up", "gamepad2", "left_trigger");
        ax_hanger_down = controllerMap.getAxisMap("hanger:axis_down", "gamepad2", "right_trigger");
        hanger.resetEncoders();
    }

    @Override
    public void update(Telemetry telemetry) {
        hanger.setPower(ax_hanger_up.get() - ax_hanger_down.get());
    }
}
