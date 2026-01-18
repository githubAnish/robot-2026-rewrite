package org.frogforce503.lib.rebuilt;

// assume turret until further updates, look in superstructure.java javadocs
// therefore first just use simple just use interpolating double tree map, then shoot on move & physics like 604 shotcalculator (have to integrate turret rotation & drivetrain rotation, both have to move)
// take into account the drivetrain pose and velocity
public final class ShotCalculator {
    private ShotCalculator() {}

    public static ShotInfo calculateHubShotInfo() {
        return new ShotInfo();
    }

    public static record ShotInfo(
        double turretAngleRad,
        double flywheelsSpeedRadPerSec,
        double hoodAngleRad,
        boolean feasibleShot
    ) {
        public ShotInfo() {
            this(0.0, 0.0, 0.0, false);
        }
    }
}