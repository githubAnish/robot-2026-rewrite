package org.frogforce503.robot.subsystems.superstructure.indexer.io;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public IndexerIOData data = new IndexerIOData(false, 0.0, 0.0, 0.0, 0.0);
    }

    record IndexerIOData(
        boolean motorConnected,
        double velocityRadPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(IndexerIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runVelocity(double velocityRadPerSec, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}
