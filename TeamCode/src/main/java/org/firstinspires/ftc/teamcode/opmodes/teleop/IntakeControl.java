package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class IntakeControl extends ControlModule{

    private Intake intake;
    private ControllerMap.AxisEntry intake_button;
    private ControllerMap.AxisEntry outtake_button;


    public IntakeControl(String name) {
        super(name);
    }


    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.intake = robot.intake;
        intake_button = controllerMap.getAxisMap("intake:right_trigger", "gamepad1","right_trigger");
        outtake_button = controllerMap.getAxisMap("intake:right_trigger_2", "gamepad2","right_trigger");

    }

    @Override
    public void update(Telemetry telemetry) {
        intake.setSpinnerPower(intake_button.get() - outtake_button.get());

    }
}
