package org.frogforce503.robot.subsystems.climberdeploy.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberDeployIO {
    @AutoLog
    class ClimberDeployIOInputs {
        public boolean motorConnected = false;
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean limitSwitchPressed = false;
    }

    default void updateInputs(ClimberDeployIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double positionMeters, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}

    default void resetEncoder() {}
}