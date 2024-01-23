package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Drone {

    private final Servo drone_launcher;

    public Drone (Servo drone_launcher) {
        this.drone_launcher = drone_launcher;
    }

    public void setLauncherPosition(double pos) {
        drone_launcher.setPosition(pos);
    }

    public double getLauncherPosition() {
        return drone_launcher.getPosition();
    }

}
