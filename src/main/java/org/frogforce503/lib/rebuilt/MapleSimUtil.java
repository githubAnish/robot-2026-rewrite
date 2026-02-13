package org.frogforce503.lib.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public final class MapleSimUtil {
    // Bump Sim Constants
    private static final double maxLinearSpeedOverBumpMetersPerSec = DriveConstants.maxLinearSpeed / 5;

    private static final Rectangle2d blueLeftBump =
        new Rectangle2d(FieldConstants.LeftBump.blueBackLeftCorner, FieldConstants.LeftBump.blueFrontRightCorner);

    private static final Rectangle2d blueRightBump =
        new Rectangle2d(FieldConstants.RightBump.blueBackLeftCorner, FieldConstants.RightBump.blueFrontRightCorner);

    private static final Rectangle2d redLeftBump =
        new Rectangle2d(FieldConstants.LeftBump.redBackLeftCorner, FieldConstants.LeftBump.redFrontRightCorner);

    private static final Rectangle2d redRightBump =
        new Rectangle2d(FieldConstants.RightBump.redBackLeftCorner, FieldConstants.RightBump.redFrontRightCorner);

    // Shoot Sim Constants
    private static final Timer shotTimer = new Timer();
    private static final Translation3d shotTolerance = new Translation3d(0.5, 0.5, 0.5);

    private MapleSimUtil() {}

    public static void initializeArena() {
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false)); // Allow MapleSim to cross over bump
    }

    // Applies max velocity to bumps instead of blocking them out like MapleSim
    public static ChassisSpeeds limitVelocityOverBumps(Translation2d robotTranslation, ChassisSpeeds robotVelocity) {
        double linearSpeed =
            Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

        boolean inBump =
            blueLeftBump.contains(robotTranslation) ||
            blueRightBump.contains(robotTranslation) ||
            redLeftBump.contains(robotTranslation) ||
            redRightBump.contains(robotTranslation);

        if ((inBump && linearSpeed <= maxLinearSpeedOverBumpMetersPerSec) || !inBump) {
            return robotVelocity;
        }

        double scalar = maxLinearSpeedOverBumpMetersPerSec / linearSpeed;

        return new ChassisSpeeds(
            robotVelocity.vxMetersPerSecond * scalar,
            robotVelocity.vyMetersPerSecond * scalar,
            robotVelocity.omegaRadiansPerSecond);
    }
    
    public static void scoreFuelIntoHub(
        Pose2d pose,
        ChassisSpeeds fieldRelativeVelocity,
        Rotation2d turretFieldRelativeAngle,
        double hoodAngleRad,
        double shotFireRateBallsPerSec
    ) {
        if (shotFireRateBallsPerSec < 0) {
            System.out.println("Balls Per Second < 0" + ErrorUtil.attachJavaClassName(MapleSimUtil.class));
            return;
        }

        double shotDelaySec = 1.0 / shotFireRateBallsPerSec;

        // Allow very first shot (timer not used yet, get() == 0.0), or when cooldown has elapsed
        if (shotTimer.isRunning() && !shotTimer.hasElapsed(shotDelaySec)) {
            return; // Cooldown not done; skip creating new projectile
        }

        GamePieceProjectile fuel =
            new RebuiltFuelOnFly(
                pose.getTranslation(),
                GeomUtil.toPose2d(pose.getRotation()).plus(GeomUtil.toTransform2d(TurretConstants.robotToTurret)).getTranslation(),
                fieldRelativeVelocity,
                turretFieldRelativeAngle, // need to change to field-relative turret angle
                Inches.of(24),
                MetersPerSecond.of(10),
                Radians.of(hoodAngleRad));

        fuel
            .withTargetPosition(() -> FieldConstants.isRed() ? FieldConstants.Hub.redShotPose : FieldConstants.Hub.blueShotPose)
            .withTargetTolerance(shotTolerance)
            .withProjectileTrajectoryDisplayCallBack(
                pose3ds -> Logger.recordOutput("GameViz/SuccessfulFuelShot", pose3ds.toArray(Pose3d[]::new)),
                pose3ds -> Logger.recordOutput("GameViz/UnsucessfulFuelShot", pose3ds.toArray(Pose3d[]::new))
            )
            .enableBecomesGamePieceOnFieldAfterTouchGround();

        SimulatedArena.getInstance().addGamePieceProjectile(fuel);

        // Restart cooldown timer after firing
        shotTimer.restart();
    }
}