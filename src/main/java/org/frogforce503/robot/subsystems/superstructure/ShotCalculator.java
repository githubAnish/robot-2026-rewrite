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

    public static final double minDistanceHubShoot = 0.8789512555744705;
    public static final double maxDistanceHubShoot = 6.011086792618746;

    public static final double minDistanceLobShoot = 4.548765387286399;
    public static final double maxDistanceLobShoot = 15.0;

    // Maps
    private static final InterpolatingDoubleTreeMap hubHoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hubFlywheelsSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hubTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap lobHoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap lobFlywheelsSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap lobTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        // Configure hub shotmaps (tuned in sim)
        hubHoodAngleMap.put(0.8789512555744705, Units.degreesToRadians(4.0));
        hubHoodAngleMap.put(1.5750158519083022, Units.degreesToRadians(15.0));
        hubHoodAngleMap.put(2.1927194507792565, Units.degreesToRadians(15.0));
        hubHoodAngleMap.put(2.8789697547312643, Units.degreesToRadians(20.0));
        hubHoodAngleMap.put(3.3990839065891034, Units.degreesToRadians(23.0));
        hubHoodAngleMap.put(4.0109712851978365, Units.degreesToRadians(24.0));
        hubHoodAngleMap.put(4.712478631465277, Units.degreesToRadians(27.0));
        hubHoodAngleMap.put(5.254298900120757, Units.degreesToRadians(30.0));
        hubHoodAngleMap.put(6.011086792618746, Units.degreesToRadians(33.0));

        hubFlywheelsSpeedMap.put(0.8789512555744705, Units.rotationsPerMinuteToRadiansPerSecond(1750.0));
        hubFlywheelsSpeedMap.put(1.5750158519083022, Units.rotationsPerMinuteToRadiansPerSecond(1750.0));
        hubFlywheelsSpeedMap.put(2.1927194507792565, Units.rotationsPerMinuteToRadiansPerSecond(1900.0));
        hubFlywheelsSpeedMap.put(2.8789697547312643, Units.rotationsPerMinuteToRadiansPerSecond(2000.0));
        hubFlywheelsSpeedMap.put(3.3990839065891034, Units.rotationsPerMinuteToRadiansPerSecond(2000.0));
        hubFlywheelsSpeedMap.put(4.0109712851978365, Units.rotationsPerMinuteToRadiansPerSecond(2100.0));
        hubFlywheelsSpeedMap.put(4.712478631465277, Units.rotationsPerMinuteToRadiansPerSecond(2150.0));
        hubFlywheelsSpeedMap.put(5.254298900120757, Units.rotationsPerMinuteToRadiansPerSecond(2200.0));
        hubFlywheelsSpeedMap.put(6.011086792618746, Units.rotationsPerMinuteToRadiansPerSecond(2250.0));

        hubTimeOfFlightMap.put(0.8789512555744705, 1.1);
        hubTimeOfFlightMap.put(1.5750158519083022, 1.0);
        hubTimeOfFlightMap.put(2.1927194507792565, 1.1);
        hubTimeOfFlightMap.put(2.8789697547312643, 1.25);
        hubTimeOfFlightMap.put(3.3990839065891034, 1.3);
        hubTimeOfFlightMap.put(4.0109712851978365, 1.2);
        hubTimeOfFlightMap.put(4.712478631465277, 1.2);
        hubTimeOfFlightMap.put(5.254298900120757, 1.19);
        hubTimeOfFlightMap.put(6.011086792618746, 1.19);

        // Configure lob shotmaps (tuned in sim)
        lobHoodAngleMap.put(4.548765387286399, Units.degreesToRadians(34.0));
        lobHoodAngleMap.put(6.5700978946700115, Units.degreesToRadians(34.0));
        lobHoodAngleMap.put(8.066151061468178, Units.degreesToRadians(34.0));
        lobHoodAngleMap.put(9.36979128257135, Units.degreesToRadians(34.0));
        lobHoodAngleMap.put(10.13710863970844, Units.degreesToRadians(34.0));
        lobHoodAngleMap.put(12.054478922470933, Units.degreesToRadians(34.0));

        lobFlywheelsSpeedMap.put(4.548765387286399, Units.rotationsPerMinuteToRadiansPerSecond(1500.0));
        lobFlywheelsSpeedMap.put(6.5700978946700115, Units.rotationsPerMinuteToRadiansPerSecond(1850.0));
        lobFlywheelsSpeedMap.put(8.066151061468178, Units.rotationsPerMinuteToRadiansPerSecond(2150.0));
        lobFlywheelsSpeedMap.put(9.36979128257135, Units.rotationsPerMinuteToRadiansPerSecond(2250.0));
        lobFlywheelsSpeedMap.put(10.13710863970844, Units.rotationsPerMinuteToRadiansPerSecond(2500.0));
        lobFlywheelsSpeedMap.put(12.054478922470933, Units.rotationsPerMinuteToRadiansPerSecond(2750.0));

        lobTimeOfFlightMap.put(4.548765387286399, 1.0);
        lobTimeOfFlightMap.put(6.5700978946700115, 1.2);
        lobTimeOfFlightMap.put(8.066151061468178, 1.3);
        lobTimeOfFlightMap.put(9.36979128257135, 1.4);
        lobTimeOfFlightMap.put(10.13710863970844, 1.7);
        lobTimeOfFlightMap.put(12.054478922470933, 2.6);
    }

    public static ShotInfo calculateShotInfo(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity, ChassisSpeeds fieldRelativeVelocity) {
        // Get inputs
        final boolean isHubShot = FieldConstants.inAllianceZone(robotPose);
        final Translation2d target = FieldConstants.getShotTarget(robotPose).toTranslation2d();
        final InterpolatingDoubleTreeMap hoodAngleMap = isHubShot ? hubHoodAngleMap : lobHoodAngleMap;
        final InterpolatingDoubleTreeMap flywheelsSpeedMap = isHubShot ? hubFlywheelsSpeedMap : lobFlywheelsSpeedMap;
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
                flywheelsSpeedMap.get(lookaheadTurretToTargetDistance),
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