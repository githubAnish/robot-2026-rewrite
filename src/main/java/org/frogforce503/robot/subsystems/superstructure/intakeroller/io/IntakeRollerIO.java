package org.frogforce503.robot.subsystems.superstructure.intakeroller.io;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
    @AutoLog
    class IntakeRollerIOInputs {
        public IntakeRollerIOData data = new IntakeRollerIOData(false, 0.0, 0.0, 0.0, 0.0);
    }

    record IntakeRollerIOData(
        boolean motorConnected,
        double velocityRadPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(IntakeRollerIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runVelocity(double velocityRadPerSec, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}
