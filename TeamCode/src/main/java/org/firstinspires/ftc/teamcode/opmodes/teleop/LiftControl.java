package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class LiftControl extends ControlModule{

    private Lift lift;
    private ControllerMap.AxisEntry ax_lift_right_y;
    private ControllerMap.ButtonEntry tray_up;
    private ControllerMap.ButtonEntry tray_down;
    private ControllerMap.ButtonEntry air_plane_launcher;

    public LiftControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        ax_lift_right_y = controllerMap.getAxisMap("lift:right_y", "gamepad2", "right_stick_y");

        tray_up = controllerMap.getButtonMap("lift:dpad_left", "gamepad1","y");
        tray_down = controllerMap.getButtonMap("lift:dpad_right", "gamepad1","a");
        air_plane_launcher = controllerMap.getButtonMap("lift:air_plane_launcher", "gamepad2","y");
    }

    @Override
    public void update(Telemetry telemetry) {

        lift.setPower(ax_lift_right_y.get()*0.5);

        if (tray_up.get()) {
            lift.setTrayPosition(0.15);
        }
        if (tray_down.get()) {
            lift.setTrayPosition(0.5);
        }

        if (air_plane_launcher.get()) {
            lift.setAPLPosition(-0.5);
        }

    }
}
