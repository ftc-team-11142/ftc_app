package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drone;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class DroneControl extends ControlModule{

    private Drone drone;
    private ControllerMap.ButtonEntry launch;

    public DroneControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.drone = robot.drone;
        launch = controllerMap.getButtonMap("drone:launch","gamepad2","dpad_left");

        drone.setLauncherPosition(0);
    }

    @Override
    public void update(Telemetry telemetry) {
        if (launch.edge() == -1) {
            drone.setLauncherPosition(0);
        }
    }
}
