package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class LiftControl extends ControlModule{

    private Lift lift;
    private ControllerMap.AxisEntry ax_lift_right_y;
    private ControllerMap.ButtonEntry arm_up;
    private ControllerMap.ButtonEntry arm_down;
    private ControllerMap.ButtonEntry launch_drone;
    private ControllerMap.ButtonEntry load_drone;

    public LiftControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        ax_lift_right_y = controllerMap.getAxisMap("lift:right_y", "gamepad2", "right_stick_y");

        arm_up = controllerMap.getButtonMap("lift:arm_up", "gamepad1","y");
        arm_down = controllerMap.getButtonMap("lift:arm_down", "gamepad1","a");
        launch_drone = controllerMap.getButtonMap("lift:launch_drone", "gamepad2","y");
        load_drone = controllerMap.getButtonMap("lift:load_drone", "gamepad2","a");
    }

    @Override
    public void update(Telemetry telemetry) {

        lift.setPower(ax_lift_right_y.get()*0.5);

        if (arm_up.get()) {
//            lift.setArmPosition();
        }
        if (arm_down.get()) {
//            lift.setArmPosition();
        }

        if (launch_drone.get()) {
            lift.setAPLPosition(0.259);
        }
        if(load_drone.get()) {
            lift.setAPLPosition(0.042);
        }

    }
}
