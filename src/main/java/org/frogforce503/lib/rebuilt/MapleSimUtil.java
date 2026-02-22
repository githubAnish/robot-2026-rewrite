package org.frogforce503.lib.rebuilt;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
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

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
    public static final double shotFireRateBallsPerSec = 7; // How many balls can you fire within 1 sec?

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

    public static void intakeFuelFromGround(SwerveDriveSimulation driveSimulation) {
        IntakeSimulation x =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel", 
                driveSimulation,
                null,
                null,
                IntakeSimulation.IntakeSide.BACK,
                24);
    }
    
    public static void scoreFuelIntoHub(
        Pose2d pose,
        ChassisSpeeds robotFieldRelativeVelocity,
        Rotation2d turretFieldRelativeAngle,
        double hoodAngleRad,
        double flywheelsSpeedRadPerSec
    ) {
        double shotDelaySec = 1.0 / shotFireRateBallsPerSec;

        // Allow very first shot (timer not used yet, get() == 0.0), or when cooldown has elapsed
        if (shotTimer.isRunning() && !shotTimer.hasElapsed(shotDelaySec)) {
            return; // Cooldown not done; skip creating new projectile
        }

        GamePieceProjectile fuel =
            new RebuiltFuelOnFly(
                pose.getTranslation().plus(TurretConstants.robotToTurret.getTranslation().toTranslation2d()),
                Translation2d.kZero, // Using TurretConstants.robotToTurret causes the initial position to shift because the GamePieceProjectile constructor rotates the position about the origin (Z axis) by the shooter’s facing. Since robotPosition is only used to compute the game piece’s initial position and isn’t used elsewhere, it can be simplified by treating the turret’s position as the robot position (i.e., using Translation.kZero)
                robotFieldRelativeVelocity,
                turretFieldRelativeAngle,
                TurretConstants.robotToTurret.plus(HoodConstants.turretToHood).getMeasureZ().plus(Inches.of(4)), // 4 inches is offset
                MetersPerSecond.of(flywheelsSpeedRadPerSec * FlywheelsConstants.kSimRadiusMeters),
                Radians.of(hoodAngleRad).plus(Degrees.of(0)));

        fuel
            .withTargetPosition(() -> FieldConstants.isRed() ? FieldConstants.Hub.redShotPose : FieldConstants.Hub.blueShotPose)
            .withTargetTolerance(shotTolerance)
            .withProjectileTrajectoryDisplayCallBack(
                pose3ds -> Logger.recordOutput("GameViz/SuccessfulFuelShot", pose3ds.toArray(Pose3d[]::new)),
                pose3ds -> Logger.recordOutput("GameViz/UnsucessfulFuelShot", pose3ds.toArray(Pose3d[]::new))
            );

        SimulatedArena.getInstance().addGamePieceProjectile(fuel);

        // Restart cooldown timer after firing
        shotTimer.restart();
    }
}