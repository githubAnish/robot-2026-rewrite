package org.frogforce503.robot.subsystems.launcher;

import java.util.Set;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.FieldConstants.Lines;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.Setter;

public class LaunchCalculator {
    private static LaunchCalculator instance;

    private double lastHoodAngle;
    private Rotation2d lastDriveAngle;

    // Cached info
    private ShotInfo latestShotInfo = null;
    @Setter @Getter private boolean isShotFeasible = false;
    @Setter @Getter private LaunchPreset shotPreset = LaunchPreset.NONE;

    // Constants
    private final LinearFilter hoodAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private final LinearFilter driveAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private final double phaseDelay = 0.03;

    public static final double minDistanceHubShoot = 0.8789512555744705;
    public static final double maxDistanceHubShoot = 6.011086792618746;

    public static final double minDistanceLobShoot = 4.548765387286399;
    public static final double maxDistanceLobShoot = 15.0;

    private static final Transform2d depotLobPoseOffset = GeomUtil.toTransform2d(Units.inchesToMeters(36), 0);
    private static final Transform2d outpostLobPoseOffset = GeomUtil.toTransform2d(Units.inchesToMeters(36), Units.inchesToMeters(18));

    // Maps
    private final InterpolatingDoubleTreeMap hubHoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hubFlywheelsSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hubTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap lobHoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap lobFlywheelsSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap lobTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    public LaunchCalculator() {
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

    public static LaunchCalculator getInstance() {
        if (instance == null) {
            instance = new LaunchCalculator();
        }
        return instance;
    }

    public double getMinTimeOfFlight() {
        return hubTimeOfFlightMap.get(minDistanceHubShoot);
    }

    public double getMaxTimeOfFlight() {
        return hubTimeOfFlightMap.get(maxDistanceHubShoot);
    }

    public static boolean inAllianceZone(Pose2d robotPose) {
        return AllianceFlipUtil.applyX(robotPose.getX()) < Lines.blueInitLineX;
    }

    private static Translation2d getDepotLobPose() {
        return
            AllianceFlipUtil.apply(
                FieldConstants.Depot.blue
                    .getCenter()
                    .plus(depotLobPoseOffset)
                    .getTranslation());
    }

    private static Translation2d getOutpostLobPose() {
        return
            AllianceFlipUtil.apply(
                FieldConstants.Outpost.blue
                    .plus(outpostLobPoseOffset)
                    .getTranslation());
    }

    /**
     * Returns the hub shot pose when the robot is in the alliance zone;
     * otherwise returns the nearest lob shot pose (Depot or Outpost).
     */
    public static Translation3d getShotTarget(Pose2d robotPose) {
        return
            inAllianceZone(robotPose)
                ? AllianceFlipUtil.apply(FieldConstants.Hub.blueShotPose)
                : new Translation3d(
                    robotPose
                        .getTranslation()
                        .nearest(Set.of(getDepotLobPose(), getOutpostLobPose())));
    }

    public ShotInfo calculateShotInfo(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity, ChassisSpeeds fieldRelativeVelocity) {
        if (latestShotInfo != null) {
            return latestShotInfo;
        }

        // Get inputs
        boolean isHubShot = inAllianceZone(robotPose);
        Translation2d target = LaunchCalculator.getShotTarget(robotPose).toTranslation2d();
        InterpolatingDoubleTreeMap hoodAngleMap = isHubShot ? hubHoodAngleMap : lobHoodAngleMap;
        InterpolatingDoubleTreeMap flywheelsSpeedMap = isHubShot ? hubFlywheelsSpeedMap : lobFlywheelsSpeedMap;
        InterpolatingDoubleTreeMap timeOfFlightMap = isHubShot ? hubTimeOfFlightMap : lobTimeOfFlightMap;

        // Calculate estimated pose while accounting for phase delay
        robotPose =
            robotPose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate target
        Pose2d launcherPosition = robotPose.transformBy(GeomUtil.toTransform2d(HoodConstants.robotToHood));
        double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

        // Calculate field relative launcher velocity
        var robotVelocity = fieldRelativeVelocity;
        var robotAngle = robotPose.getRotation();

        ChassisSpeeds launcherVelocity =
            DriverStation.isAutonomous()
                ? robotVelocity
                : GeomUtil.transformVelocity(
                        robotVelocity,
                        HoodConstants.robotToHood.getTranslation().toTranslation2d(),
                        robotAngle);

        // Account for imparted velocity by robot (launcher) to offset
        double timeOfFlight = timeOfFlightMap.get(launcherToTargetDistance);
        Pose2d lookaheadPose = launcherPosition;
        double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadLauncherToTargetDistance);

            double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
            double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;

            lookaheadPose =
                new Pose2d(
                    launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    launcherPosition.getRotation());

            lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Account for launcher being off center
        Pose2d lookaheadRobotPose = lookaheadPose.transformBy(GeomUtil.toTransform2d(HoodConstants.robotToHood).inverse());
        Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

        // Calculate remaining parameters
        double hoodAngle = hoodAngleMap.get(lookaheadLauncherToTargetDistance);

        if (lastDriveAngle == null) {
            lastDriveAngle = driveAngle;
        }

        if (Double.isNaN(lastHoodAngle)) {
            lastHoodAngle = hoodAngle;
        }

        double hoodVelocity =
            hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);

        double driveVelocity =
            driveAngleFilter.calculate(
                driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);

        lastHoodAngle = hoodAngle;
        lastDriveAngle = driveAngle;

        // Update latest shot info
        latestShotInfo =
            new ShotInfo(
                driveAngle,
                driveVelocity,
                hoodAngle,
                hoodVelocity,
                flywheelsSpeedMap.get(lookaheadLauncherToTargetDistance),
                lookaheadLauncherToTargetDistance);

        // Log data
        Logger.recordOutput("ShotCalculator/Is Hub Shot?", isHubShot);
        Logger.recordOutput("ShotCalculator/TargetTranslation", target);

        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadRobotPose);
        Logger.recordOutput("ShotCalculator/LatestShotInfo", latestShotInfo);
        Logger.recordOutput("ShotCalculator/Is Shot Feasible?", isShotFeasible);

        Logger.recordOutput("ShotCalculator/Shot Preset", shotPreset);

        return latestShotInfo;
    }

    public void clearLatestShotInfo() {
        latestShotInfo = null;
    }

    private static Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
        Rotation2d fieldToTargetAngle =
            target
                .minus(robotPose.getTranslation())
                .getAngle();

        Rotation2d offsetCorrection =
            new Rotation2d(
                Math.asin(
                    MathUtil.clamp(
                        HoodConstants.robotToHood.getTranslation().getY() / target.getDistance(robotPose.getTranslation()),
                        -1.0,
                        1.0)));
                        
        return
            fieldToTargetAngle
                .plus(offsetCorrection)
                .plus(HoodConstants.robotToHood.getRotation().toRotation2d())
                .plus(Rotation2d.kPi); // launcher is on opposite side of front
    }

    public record ShotInfo(
        Rotation2d driveAngle,
        double driveVelocity,
        double hoodAngleRad,
        double hoodVelocityRadPerSec,
        double flywheelsVelocityRadPerSec,
        double launcherToTargetDistance) {}
}