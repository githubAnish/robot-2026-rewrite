package org.frogforce503.lib.motorcontrol;

import edu.wpi.first.math.controller.PIDController;

public record PIDConfig(
    double kP,
    double kI,
    double kD,
    double kIZone,
    double kMinimumIntegral,
    double kMaximumIntegral
) {
    public PIDConfig(double kP, double kI, double kD, double kIZone) {
        this(kP, kI, kD, kIZone, -1.0, 1.0);
    }

    public PIDConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, Double.POSITIVE_INFINITY);
    }

    public PIDConfig() {
        this(0.0, 0.0, 0.0);
    }

    public PIDController toPIDController() {
        PIDController controller = new PIDController(kP, kI, kD);
        controller.setIZone(kIZone);
        controller.setIntegratorRange(kMinimumIntegral, kMaximumIntegral);
        return controller;
    }
}