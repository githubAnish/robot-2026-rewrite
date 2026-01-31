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
    
    public static TurretSetpoint calculateTurretRobotRelativeSetpoint(
        Rotation2d turretFieldAngle,
        double turretFieldAngularVelocity,
        Rotation2d robotAngle,
        double robotOmega
    ) {
        double robotRelativeAngle = turretFieldAngle.minus(robotAngle).getRadians();
        double robotRelativeVelocity = turretFieldAngularVelocity - robotOmega;

        return new TurretSetpoint(robotRelativeAngle, robotRelativeVelocity);
    }

    public record TurretSetpoint(
        double angleRad,
        double velocityRadPerSec) {}

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