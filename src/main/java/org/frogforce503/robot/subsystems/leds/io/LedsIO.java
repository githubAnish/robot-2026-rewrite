package org.frogforce503.robot.subsystems.leds.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

public interface LedsIO {
    @AutoLog
    class LedsIOInputs {
        public boolean stripConnected = false;
        public String patternName = "";
        public double supplyVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(LedsIOInputs inputs) {}

    default void runPattern(ControlRequest pattern) {}
}