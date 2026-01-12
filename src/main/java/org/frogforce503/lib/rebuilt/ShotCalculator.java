package org.frogforce503.lib.rebuilt;

// assume no turret until further updates, look in superstructure.java javadocs
// therefore don't do shoot on move and just use interpolating double tree map / physics like 604 shotcalculator
// take into account the drivetrain pose and velocity
public final class ShotCalculator {
    private ShotCalculator() {}

    public static ShotInfo calculateShotInfo() {
        return new ShotInfo();
    }

    public static record ShotInfo(
        double flywheelsSpeed,
        double hoodAngleRad,
        boolean feasibleShot
    ) {
        public ShotInfo() {
            this(0.0, 0.0, false);
        }
    }
}
