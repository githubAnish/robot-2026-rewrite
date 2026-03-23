package org.frogforce503.robot.subsystems.superstructure;

import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;

import lombok.Getter;

public enum ShotPreset {
    BATTER(HoodConstants.BATTER, FlywheelsConstants.BATTER), // Up against hub
    TRENCH(HoodConstants.TRENCH, FlywheelsConstants.TRENCH), // Robot center on initiation line
    DEPOT(HoodConstants.DEPOT, FlywheelsConstants.DEPOT); // From front corners of depot

    @Getter private final double hoodAngleRad;
    @Getter private final double flywheelsVelocityRadPerSec;

    private ShotPreset(double hoodAngleRad, double flywheelsVelocityRadPerSec) {
        this.hoodAngleRad = hoodAngleRad;
        this.flywheelsVelocityRadPerSec = flywheelsVelocityRadPerSec;
    }
}