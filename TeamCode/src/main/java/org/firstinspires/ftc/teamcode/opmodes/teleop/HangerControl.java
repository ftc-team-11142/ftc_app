package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Hanger;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class HangerControl extends ControlModule{

    private Hanger hanger;
    private ControllerMap.AxisEntry ax_lift_left_y;

    public HangerControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.hanger = robot.hanger;
        ax_lift_left_y = controllerMap.getAxisMap("hanger:left_y_hanger", "gamepad2", "left_stick_y");
        hanger.resetEncoders();
    }

    @Override
    public void update(Telemetry telemetry) {
        hanger.setPower(ax_lift_left_y.get());
    }
}
