package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class IntakeControl extends ControlModule{

    private Intake intake;
    private ControllerMap.ButtonEntry intake_button;
    private Boolean claw_open = true;

    public IntakeControl(String name) {
        super(name);
    }


    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.intake = robot.intake;
        intake_button = controllerMap.getButtonMap("intake:right_trigger", "gamepad1","x");

    }

    @Override
    public void update(Telemetry telemetry) {
        if (intake_button.edge() == -1) {
            claw_open = !claw_open;
        }

        if (claw_open) {
            intake.setClawleftPosition(0.75);
            intake.setClawRightPosition(0.25);
        }
        else {
            intake.setClawleftPosition(1);
            intake.setClawRightPosition(0.0);
        }

    }
}
