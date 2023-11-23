package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class IntakeControl extends ControlModule{

    private Intake intake;
    private ControllerMap.AxisEntry right_trigger;
    private ControllerMap.ButtonEntry x_button;
    private ControllerMap.ButtonEntry b_button;

    public IntakeControl(String name) {
        super(name);
    }


    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.intake = robot.intake;
        right_trigger = controllerMap.getAxisMap("intake:right_trigger", "gamepad1","right_trigger");

        x_button = controllerMap.getButtonMap("intake:x_button", "gamepad1","x");
        b_button = controllerMap.getButtonMap("intake:b_button", "gamepad1","b");
        intake.setRotatorPosition(0);
    }

    @Override
    public void update(Telemetry telemetry) {
        intake.setSpinnerPower(right_trigger.get());

        if (x_button.get()) {
            intake.setRotatorPosition(0);
        }

        if (b_button.get()) {
            intake.setRotatorPosition(0.5);
        }
    }
}
