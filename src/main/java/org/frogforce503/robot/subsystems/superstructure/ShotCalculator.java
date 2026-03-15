package org.frogforce503.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
    private static Rotation2d lastTurretAngle;
    private static double lastHoodAngle;
    private static Rotation2d turretAngle;
    private static double hoodAngle = Double.NaN;
    private static double turretVelocity;
    private static double hoodVelocity;

    // Constants
    private static final LinearFilter turretAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private static final LinearFilter hoodAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private static final double phaseDelay = 0.03;

    public static final double minDistanceHubShoot = 1.205; // Update based on shotmap
    public static final double maxDistanceHubShoot = 5.427;

    public static final double minDistanceLobShoot = 0.0; // Update based on shotmap distance range
    public static final double maxDistanceLobShoot = 20.0; // Update based on shotmap distance range

    // Maps
    private static final InterpolatingDoubleTreeMap hubHoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hubFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hubTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap lobHoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap lobFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap lobTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        // Configure hub shotmaps (tuned in sim)
        hubHoodAngleMap.put(1.205, Units.degreesToRadians(10));
        hubHoodAngleMap.put(2.056, Units.degreesToRadians(20));
        hubHoodAngleMap.put(2.585, Units.degreesToRadians(26));
        hubHoodAngleMap.put(2.905, Units.degreesToRadians(30));
        hubHoodAngleMap.put(3.110, Units.degreesToRadians(33));
        hubHoodAngleMap.put(3.716, Units.degreesToRadians(35));
        hubHoodAngleMap.put(4.360, Units.degreesToRadians(36));
        hubHoodAngleMap.put(4.950, Units.degreesToRadians(40));
        hubHoodAngleMap.put(5.427, Units.degreesToRadians(41));

        hubFlywheelSpeedMap.put(1.263, Units.rotationsPerMinuteToRadiansPerSecond(1600));
        hubFlywheelSpeedMap.put(2.056, Units.rotationsPerMinuteToRadiansPerSecond(1600));
        hubFlywheelSpeedMap.put(2.585, Units.rotationsPerMinuteToRadiansPerSecond(1650));
        hubFlywheelSpeedMap.put(2.905, Units.rotationsPerMinuteToRadiansPerSecond(1750));
        hubFlywheelSpeedMap.put(3.110, Units.rotationsPerMinuteToRadiansPerSecond(1775));
        hubFlywheelSpeedMap.put(3.716, Units.rotationsPerMinuteToRadiansPerSecond(1900));
        hubFlywheelSpeedMap.put(4.360, Units.rotationsPerMinuteToRadiansPerSecond(2000));
        hubFlywheelSpeedMap.put(4.950, Units.rotationsPerMinuteToRadiansPerSecond(2100));
        hubFlywheelSpeedMap.put(5.427, Units.rotationsPerMinuteToRadiansPerSecond(2200));

        hubTimeOfFlightMap.put(1.263, 0.62);
        hubTimeOfFlightMap.put(2.585, 0.71);
        hubTimeOfFlightMap.put(3.110, 0.75);
        hubTimeOfFlightMap.put(4.360, 0.95);
        hubTimeOfFlightMap.put(5.427, 1.1);

        // Configure lob shotmaps (tuned in sim)
        lobHoodAngleMap.put(8.095, Units.degreesToRadians(45));
        lobHoodAngleMap.put(9.861, Units.degreesToRadians(45));

        lobFlywheelSpeedMap.put(8.095, Units.rotationsPerMinuteToRadiansPerSecond(2000));
        lobFlywheelSpeedMap.put(9.861, Units.rotationsPerMinuteToRadiansPerSecond(2500));

        lobTimeOfFlightMap.put(8.095, 1.2);
        lobTimeOfFlightMap.put(9.861, 1.4);
    }

    public static ShotInfo calculateShotInfo(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity, ChassisSpeeds fieldRelativeVelocity) {
        // Get inputs
        final boolean isHubShot = FieldConstants.inAllianceZone(robotPose);
        final Translation2d target = FieldConstants.getShotTarget(robotPose).toTranslation2d();
        final InterpolatingDoubleTreeMap hoodAngleMap = isHubShot ? hubHoodAngleMap : lobHoodAngleMap;
        final InterpolatingDoubleTreeMap flywheelSpeedMap = isHubShot ? hubFlywheelSpeedMap : lobFlywheelSpeedMap;
        final InterpolatingDoubleTreeMap timeOfFlightMap = isHubShot ? hubTimeOfFlightMap : lobTimeOfFlightMap;

        // Calculate estimated pose while accounting for phase delay
        robotPose =
            robotPose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        Pose2d turretPosition = robotPose.plus(GeomUtil.toTransform2d(TurretConstants.robotToTurret));
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = fieldRelativeVelocity;
        double robotAngle = robotPose.getRotation().getRadians();
        
        double turretVelocityX =
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.robotToTurret.getY() * Math.cos(robotAngle)
                        - TurretConstants.robotToTurret.getX() * Math.sin(robotAngle));

        double turretVelocityY =
            robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.robotToTurret.getX() * Math.cos(robotAngle)
                        - TurretConstants.robotToTurret.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        
        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);

            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;

            lookaheadPose =
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());

            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        hoodAngle = hoodAngleMap.get(lookaheadTurretToTargetDistance);

        if (lastTurretAngle == null) {
            lastTurretAngle = turretAngle;
        }

        if (Double.isNaN(lastHoodAngle)) {
            lastHoodAngle = hoodAngle;
        }

        turretVelocity =
            turretAngleFilter.calculate(
                turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);

        hoodVelocity =
            hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);

        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;

        ShotInfo shotInfo =
            new ShotInfo(
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                flywheelSpeedMap.get(lookaheadTurretToTargetDistance),
                lookaheadTurretToTargetDistance);

        // Log calculated values
        Logger.recordOutput("ShotCalculator/TargetTranslation", target);
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return shotInfo;
    }

    public record ShotInfo(
        Rotation2d turretFieldRelativeAngle,
        double turretVelocityRadPerSec,
        double hoodAngleRad,
        double hoodVelocityRadPerSec,
        double flywheelsVelocityRadPerSec,
        double turretToTargetDistance) {}
}