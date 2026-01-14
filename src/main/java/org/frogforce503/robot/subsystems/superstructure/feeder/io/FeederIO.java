package org.frogforce503.robot.subsystems.superstructure.feeder.io;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public FeederIOData data = new FeederIOData(false, 0.0, 0.0, 0.0, 0.0);
    }

    record FeederIOData(
        boolean motorConnected,
        double velocityRadPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(FeederIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runVelocity(double velocityRadPerSec, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}
