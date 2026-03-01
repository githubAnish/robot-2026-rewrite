package org.frogforce503.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
    private static final LinearFilter turretAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private static final LinearFilter hoodAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private static Rotation2d lastTurretAngle;
    private static double lastHoodAngle;
    private static Rotation2d turretAngle;
    private static double hoodAngle = Double.NaN;
    private static double turretVelocity;
    private static double hoodVelocity;

    public static final double minDistanceToHub = 1.263; // Update based on shotmap distance range
    public static final double maxDistanceToHub = 5.427; // Update based on shotmap distance range
    private static final double phaseDelay = 0.03;

    private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
        
    private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap hubTimeOfFlightMap =
        new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap lobTimeOfFlightMap =
        new InterpolatingDoubleTreeMap();

    static {
        // Configure shotmaps (tuned in sim)
        launchHoodAngleMap.put(1.263, Rotation2d.fromDegrees(80));
        launchHoodAngleMap.put(2.056, Rotation2d.fromDegrees(70));
        launchHoodAngleMap.put(2.585, Rotation2d.fromDegrees(64));
        launchHoodAngleMap.put(2.905, Rotation2d.fromDegrees(60));
        launchHoodAngleMap.put(3.110, Rotation2d.fromDegrees(57));
        launchHoodAngleMap.put(3.716, Rotation2d.fromDegrees(55));
        launchHoodAngleMap.put(4.360, Rotation2d.fromDegrees(54));
        launchHoodAngleMap.put(4.950, Rotation2d.fromDegrees(50));
        launchHoodAngleMap.put(5.427, Rotation2d.fromDegrees(49));

        launchFlywheelSpeedMap.put(1.263, Units.rotationsPerMinuteToRadiansPerSecond(1600));
        launchFlywheelSpeedMap.put(2.056, Units.rotationsPerMinuteToRadiansPerSecond(1600));
        launchFlywheelSpeedMap.put(2.585, Units.rotationsPerMinuteToRadiansPerSecond(1650));
        launchFlywheelSpeedMap.put(2.905, Units.rotationsPerMinuteToRadiansPerSecond(1750));
        launchFlywheelSpeedMap.put(3.110, Units.rotationsPerMinuteToRadiansPerSecond(1775));
        launchFlywheelSpeedMap.put(3.716, Units.rotationsPerMinuteToRadiansPerSecond(1900));
        launchFlywheelSpeedMap.put(4.360, Units.rotationsPerMinuteToRadiansPerSecond(2000));
        launchFlywheelSpeedMap.put(4.950, Units.rotationsPerMinuteToRadiansPerSecond(2100));
        launchFlywheelSpeedMap.put(5.427, Units.rotationsPerMinuteToRadiansPerSecond(2200));

        hubTimeOfFlightMap.put(1.263, 0.62);
        hubTimeOfFlightMap.put(2.585, 0.71);
        hubTimeOfFlightMap.put(3.110, 0.75);
        hubTimeOfFlightMap.put(4.360, 0.95);
        hubTimeOfFlightMap.put(5.427, 1.1);
    }

    public static ShotInfo calculateHubShotInfo(
        Pose2d pose,
        ChassisSpeeds robotRelativeVelocity,
        ChassisSpeeds fieldRelativeVelocity
    ) {
        return
            calculateShotInfo(
                pose,
                robotRelativeVelocity,
                fieldRelativeVelocity,
                FieldConstants.isRed() 
                    ? FieldConstants.Hub.redShotPose.toTranslation2d() 
                    : FieldConstants.Hub.blueShotPose.toTranslation2d(),
                hubTimeOfFlightMap);
    }

    public static ShotInfo calculateLobShotInfo(
        Pose2d pose,
        ChassisSpeeds robotRelativeVelocity,
        ChassisSpeeds fieldRelativeVelocity
    ) {
        return
            calculateShotInfo(
                pose,
                robotRelativeVelocity,
                fieldRelativeVelocity,
                FieldConstants.isRed() 
                    ? Translation2d.kZero 
                    : Translation2d.kZero,
                lobTimeOfFlightMap);
    }

    private static ShotInfo calculateShotInfo(
        Pose2d pose,
        ChassisSpeeds robotRelativeVelocity,
        ChassisSpeeds fieldRelativeVelocity,
        Translation2d target,
        InterpolatingDoubleTreeMap timeOfFlightMap
    ) {
        // Calculate estimated pose while accounting for phase delay
        pose =
            pose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        Pose2d turretPosition = pose.transformBy(GeomUtil.toTransform2d(TurretConstants.robotToTurret));
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = fieldRelativeVelocity;
        double robotAngle = pose.getRotation().getRadians();
        
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
            timeOfFlight = hubTimeOfFlightMap.get(lookaheadTurretToTargetDistance);

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
        hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();

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

        ShotInfo latestInfo =
            new ShotInfo(
                turretToTargetDistance,
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        // Log calculated values
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return latestInfo;
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

    public record ShotInfo(
        double turretToTargetDistance,
        Rotation2d turretFieldRelativeAngle,
        double turretVelocityRadPerSec,
        double hoodAngleRad,
        double hoodVelocityRadPerSec,
        double flywheelsVelocityRadPerSec) {}

    public record TurretSetpoint(
        double angleRad,
        double velocityRadPerSec) {}
}