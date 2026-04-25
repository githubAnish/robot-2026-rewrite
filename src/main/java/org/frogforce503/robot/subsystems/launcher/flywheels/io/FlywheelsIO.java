package org.frogforce503.robot.subsystems.launcher.flywheels.io;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
    @AutoLog
    class FlywheelsIOInputs {
        public boolean leaderConnected = false;
        public double leaderVelocityRadPerSec = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderStatorCurrentAmps = 0.0;
        public double leaderTempCelsius = 0.0;

        public boolean followerConnected = false;
        public double followerVelocityRadPerSec = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerStatorCurrentAmps = 0.0;
        public double followerTempCelsius = 0.0;
    }

    default void updateInputs(FlywheelsIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runVelocity(double velocityRadPerSec, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}
