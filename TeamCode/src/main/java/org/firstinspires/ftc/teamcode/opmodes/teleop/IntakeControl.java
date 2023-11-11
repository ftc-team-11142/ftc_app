package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class IntakeControl extends ControlModule{

    private Intake intake;
    private ControllerMap.AxisEntry right_trigger;
    private ControllerMap.ButtonEntry dpad_left;
    private ControllerMap.ButtonEntry dpad_right;

    private double rot_pos = 0;

    public IntakeControl(String name) {
        super(name);
    }


    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.intake = robot.intake;
        right_trigger = controllerMap.getAxisMap("intake:right_trigger", "gamepad1","right_trigger");

        dpad_left = controllerMap.getButtonMap("intake:dpad_left", "gamepad1","dpad_left");
        dpad_right = controllerMap.getButtonMap("intake:dpad_right", "gamepad1","dpad_right");

    }

    @Override
    public void update(Telemetry telemetry) {
        intake.setSpinnerPower(right_trigger.get());

        if (dpad_left.get()) {
            rot_pos -= 0.1;
        }
        if (dpad_right.get()) {
            rot_pos += 0.1;
        }

        intake.setRotatorPosition(rot_pos);
    }
}
