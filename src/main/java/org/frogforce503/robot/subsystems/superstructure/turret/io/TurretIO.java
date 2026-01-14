package org.frogforce503.robot.subsystems.superstructure.turret.io;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public TurretIOData data = new TurretIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record TurretIOData(
        boolean motorConnected,
        double positionRad,
        double absolutePositionRad,
        double velocityRadPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(TurretIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double positionRad, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}

    default void setRelativePosition(double positionRad) {}
}