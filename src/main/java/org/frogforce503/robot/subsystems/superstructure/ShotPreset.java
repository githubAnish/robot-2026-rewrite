package org.frogforce503.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;

public enum ShotPreset {
    NONE,
    BATTER(Rotation2d.fromDegrees(0), 0, 0), // up against hub
    LOB_FROM_NZ(Rotation2d.fromDegrees(0), 0, 0), // shoots to right corner
    TOWER(Rotation2d.fromDegrees(0), 0, 0); // from center of tower

    @Getter private final Rotation2d turretFieldRelativeAngle;
    @Getter private final double flywheelsVelocityRadPerSec;
    @Getter private final double hoodAngleRad;

    private ShotPreset(Rotation2d turretFieldRelativeAngle, double flywheelsVelocityRadPerSec, double hoodAngleRad) {
        this.turretFieldRelativeAngle = turretFieldRelativeAngle;
        this.flywheelsVelocityRadPerSec = flywheelsVelocityRadPerSec;
        this.hoodAngleRad = hoodAngleRad;
    }

    private ShotPreset() {
        this(Rotation2d.kZero, 0.0, 0.0);
    }
}