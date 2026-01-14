package org.frogforce503.robot.subsystems.superstructure.intakepivot.io;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
    @AutoLog
    class IntakePivotIOInputs {
        public IntakePivotIOData data = new IntakePivotIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record IntakePivotIOData(
        boolean motorConnected,
        double positionRad,
        double velocityRadPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(IntakePivotIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double positionRad, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}
