package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

@Config //TODO
public class IntakeControl extends ControlModule{

    private Intake intake;
    private ControllerMap.ButtonEntry intake_button;
    private ControllerMap.ButtonEntry claw_high_button;
    private ControllerMap.ButtonEntry claw_mid_button;
    private ControllerMap.ButtonEntry claw_down_button;

    private boolean claw_high = false;
    private boolean claw_mid = false;
    private boolean claw_down = true;
    private boolean claw_open = true;

    private String previous_state = "claw_down";

    private ElapsedTime sequencing_timer = new ElapsedTime();

    public IntakeControl(String name) {
        super(name);
    }

    public static double mid_wrist = 0.4;
    public static double high_wrist = 0.4;
    public static double high_delay_wrist = 1.8;
    public static double mid_delay_wrist = 1.8;


    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.intake = robot.intake;
        intake_button = controllerMap.getButtonMap("intake:claw", "gamepad1","x");
        claw_high_button = controllerMap.getButtonMap("claw:high", "gamepad2", "y");
        claw_mid_button = controllerMap.getButtonMap("claw:mid", "gamepad2", "b");
        claw_down_button = controllerMap.getButtonMap("claw:down", "gamepad2", "a");
        sequencing_timer.reset();
    }

    @Override
    public void init_loop(Telemetry telemetry) {
        super.init_loop(telemetry);
        if (sequencing_timer.seconds() > 1) {
            intake.setWristPosition(0.958);
        }
    }

    @Override
    public void update(Telemetry telemetry) {
        if (intake_button.edge() == -1) {
            claw_open = !claw_open;
        }

        if (claw_open) {
            intake.setLeftPosition(0.42);
            intake.setRightPosition(0.80);
        }
        else {
            intake.setLeftPosition(0.32);
            intake.setRightPosition(1);
        }

        if (claw_high_button.edge() == -1) {

            if (claw_mid) {
                previous_state = "claw_mid";
            }
            if (claw_down) {
                previous_state = "claw_down";
            }

            claw_high = true;
            claw_mid = false;
            claw_down = false;

            claw_open = false;

            sequencing_timer.reset();
        }

        if (claw_mid_button.edge() == -1) {

            if (claw_high) {
                previous_state = "claw_high";
            }
            if (claw_down) {
                previous_state = "claw_down";
            }

            claw_high = false;
            claw_mid = true;
            claw_down = false;

            claw_open = false;
            sequencing_timer.reset();
        }

        if (claw_down_button.edge() == -1) {
            if (claw_high) {
                previous_state = "claw_high";
            }
            if (claw_mid) {
                previous_state = "claw_mid";
            }

            claw_high = false;
            claw_mid = false;
            claw_down = true;

            claw_open = true;
            sequencing_timer.reset();
        }

        if (claw_high) {
            if ((sequencing_timer.seconds() > high_delay_wrist) && previous_state.equals("claw_down")) { //TODO sequencing delays based of servo stuff
                intake.setWristPosition(high_wrist);
            }
            else {
                intake.setWristPosition(high_wrist);
            }
        }

        if (claw_mid) {
            if ((sequencing_timer.seconds() > mid_delay_wrist) && previous_state.equals("claw_down")) {
                intake.setWristPosition(mid_wrist);
            }
            else {
                intake.setWristPosition(mid_wrist);
            }
        }

        if (claw_down) {
                intake.setWristPosition(0.92);
        }

    }
}
