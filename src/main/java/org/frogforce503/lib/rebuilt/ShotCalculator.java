package org.frogforce503.lib.rebuilt;

import org.frogforce503.robot.constants.field.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotCalculator {
    private ShotCalculator() {}

    public static ShotInfo calculateHubShotInfo(ShotParameters parameters) {
        Translation3d targetHubPose =
            FieldConstants.isRed() 
                ? FieldConstants.Hub.redShotPose 
                : FieldConstants.Hub.blueShotPose;

        return new ShotInfo();
    }
    
    public static double calculateTurretRobotRelativeAngle() {
        return 0.0;
    }

    public static record ShotParameters(
        Pose2d robotPose,
        ChassisSpeeds robotRelativeVelocity
    ) {
        public ShotParameters() {
            this(Pose2d.kZero, new ChassisSpeeds());
        }
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