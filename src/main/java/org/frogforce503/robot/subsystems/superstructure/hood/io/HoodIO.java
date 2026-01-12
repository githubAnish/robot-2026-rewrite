package org.frogforce503.robot.subsystems.superstructure.hood.io;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public HoodIOData data = new HoodIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record HoodIOData(
        boolean motorConnected,
        double positionRad,
        double velocityRadPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(HoodIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double positionRad, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}