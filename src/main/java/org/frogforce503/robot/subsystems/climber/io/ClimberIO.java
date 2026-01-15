package org.frogforce503.robot.subsystems.climber.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public ClimberIOData data = new ClimberIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, false);
    }

    record ClimberIOData(
        boolean motorConnected,
        double positionMeters,
        double velocityMetersPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius,
        boolean limitSwitchPressed) {}

    default void updateInputs(ClimberIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double positionMeters, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}

    default void resetEncoder() {}
}