package org.frogforce503.robot.subsystems.superstructure;

import org.frogforce503.robot.constants.field.FieldConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class ShotCalculator {
    private ShotCalculator() {}

    public static ShotInfo calculateHubShotInfo() {
        Translation3d targetHubPose =
            FieldConstants.isRed() 
                ? FieldConstants.Hub.redShotPose 
                : FieldConstants.Hub.blueShotPose;

        return new ShotInfo();
    }
    
    public static double calculateTurretRobotRelativeAngle(double turretFieldRelativeAngleRad, Rotation2d robotAngle, double robotAngularVelocity) {
        return 0.0;
    }

    public static record ShotInfo(
        double turretRobotRelativeAngleRad,
        double flywheelsSpeedRadPerSec,
        double hoodAngleRad,
        boolean feasibleShot
    ) {
        public ShotInfo() {
            this(0.0, 0.0, 0.0, false);
        }
    }
}