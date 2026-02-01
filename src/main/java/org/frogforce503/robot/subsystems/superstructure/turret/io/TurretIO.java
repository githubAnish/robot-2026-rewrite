package org.frogforce503.robot.subsystems.superstructure.turret.io;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean motorConnected = false;
        public double positionRad = 0.0;
        public double absolutePositionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double positionRad, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}

    default void setRelativePosition(double positionRad) {}
}