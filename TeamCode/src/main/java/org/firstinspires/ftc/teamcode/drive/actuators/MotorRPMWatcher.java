package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorRPMWatcher {

    private DcMotorEx motor;
    private double ticksPerRev;
    private double targetRPM;

    public MotorRPMWatcher(DcMotorEx motor, double ticksPerRev, double targetRPM) {
        this.motor = motor;
        this.ticksPerRev = ticksPerRev;
        this.targetRPM = targetRPM;
    }

    // Calcula RPM atual
    public double getRPM() {
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond * 60.0) / ticksPerRev;
    }

    // Checa se passou do limite
    public boolean isAboveTarget() {
        return getRPM() >= targetRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }
}
