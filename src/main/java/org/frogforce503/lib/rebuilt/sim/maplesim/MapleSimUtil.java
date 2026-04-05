package org.frogforce503.lib.rebuilt.sim.maplesim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;

public class MapleSimUtil {
    // Arena Constants
    private static final FFArena2026Rebuilt arena = new FFArena2026Rebuilt(false);
    
    // Intake Constants
    private static final Distance intakeWidth = Inches.of(25.5);
    private static final Distance intakeLengthExtended = Inches.of(9.5);
    private static final int fuelCapacity = 40;

    // Shoot Constants
    private static final Translation3d shotTolerance = new Translation3d(0.2, 0.2, 0.2);
    private static final Transform2d initialFuelPositionOffset = new Transform2d(Units.inchesToMeters(3), 0, Rotation2d.kZero);
    private static final Transform3d initialShotHeightOffset = new Transform3d(0, 0, Units.inchesToMeters(4), Rotation3d.kZero);
    private static final double fuelReleasedPerShot = 4; // How many balls are fired at once?
    private static final double leftMostFuelPositionOffset = Units.inchesToMeters(-8);
    private static final double rightMostFuelPositionOffset = Units.inchesToMeters(10);
    private static final double shooterFireRateBallsPerSec = 7; // How many balls can shooter fire within 1 sec?

    private MapleSimUtil() {}
    
    public static void initializeArena() {
        SimulatedArena.overrideInstance(arena); // Allow MapleSim to cross over bump
    }

    public static void hpThrowFromOutpost() {
        arena.outpostThrowForGoal(!FieldConstants.isRed());
    }

    public static IntakeSimulation createIntake(SwerveDriveSimulation driveSimulation) {
        return
            IntakeSimulation.OverTheBumperIntake(
                "Fuel", 
                driveSimulation,
                intakeWidth,
                intakeLengthExtended,
                IntakeSimulation.IntakeSide.FRONT,
                fuelCapacity);
    }

    private static int computeFuelToShoot(int available) {
        if (available <= 0) {
            return 0;
        }

        double fillRatio = (double) available / 40.0; // Normalize (0 → 1)
        double curvedFill = Math.pow(fillRatio, 0.35); // Smooth curve
        double scaledMax = fuelReleasedPerShot * curvedFill; // Scale burst size

        // Bounds
        int minShot = Math.max(1, (int) Math.floor(scaledMax * 0.5));
        int maxShot = Math.max(1, (int) Math.ceil(scaledMax));

        // Weighted randomness
        double bias = curvedFill;
        double rand = Math.random();
        double weightedRand = (rand * (1 - bias)) + (Math.pow(rand, 0.5) * bias);

        int fuelToShoot = minShot + (int) (weightedRand * (maxShot - minShot + 1));

        // Simulate indexing inconsistency
        double misfeedChance = 0.15 * (1.0 - fillRatio);
        if (Math.random() < misfeedChance) {
            fuelToShoot -= 1;
        }

        double doubleFeedChance = 0.08 * fillRatio;
        if (Math.random() < doubleFeedChance) {
            fuelToShoot += 1;
        }

        double stutterChance = 0.1;
        if (Math.random() < stutterChance) {
            fuelToShoot += Math.random() < 0.5 ? -1 : 1;
        }

        return MathUtil.clamp(fuelToShoot, 1, available);
    }

    private static void createFuelProjectile(
        Pose2d robotPose,
        ChassisSpeeds robotFieldRelativeVelocity,
        double hoodAngleRad,
        double flywheelsSpeedRadPerSec,
        Transform2d fuelLaunchPositionOffset,
        Runnable addFuelShotInMatch
    ) {
        GamePieceProjectile fuel =
            new RebuiltFuelOnFly(
                robotPose
                    .plus(GeomUtil.toTransform2d(HoodConstants.robotToHood))
                    .plus(initialFuelPositionOffset)
                    .plus(fuelLaunchPositionOffset)
                    .getTranslation(),
                Translation2d.kZero,
                robotFieldRelativeVelocity,
                robotPose
                    .getRotation()
                    .plus(Rotation2d.kPi), // launcher is on opposite side of front
                Pose3d.kZero
                    .plus(HoodConstants.robotToHood)
                    .plus(initialShotHeightOffset)
                    .getMeasureZ(),
                MetersPerSecond.of(flywheelsSpeedRadPerSec * FlywheelsConstants.kSimRadiusMeters),
                Radians.of(Units.degreesToRadians(90) - hoodAngleRad)); // 0 deg hood = 90 deg shot angle (since shots have to go up) & vice versa

        fuel
            .withTargetPosition(() -> FieldConstants.getShotTarget(robotPose))
            .withTargetTolerance(shotTolerance)
            .withProjectileTrajectoryDisplayCallBack(
                pose3ds -> Logger.recordOutput("GameViz/SuccessfulFuelShot", pose3ds.toArray(Pose3d[]::new)),
                pose3ds -> Logger.recordOutput("GameViz/UnsucessfulFuelShot", pose3ds.toArray(Pose3d[]::new))
            )
            .setHitTargetCallBack(addFuelShotInMatch);

        SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }

    public static void shootFuel(
        Pose2d robotPose,
        ChassisSpeeds fieldRelativeVelocity,
        double hoodAngleRad,
        double flywheelsSpeedRadPerSec,
        IntakeSimulation intakeSimulation,
        Timer shotTimer,
        boolean needFuelFromIntakeForShoot,
        Runnable onScore
    ) {
        int available = intakeSimulation.getGamePiecesAmount();

        if (needFuelFromIntakeForShoot && available <= 0) {
            return; // Don't shoot balls if there are none
        }

        double shotDelaySec = 1.0 / shooterFireRateBallsPerSec;

        // Allow very first shot (timer not used yet, get() == 0.0), or when cooldown has elapsed
        if (shotTimer.isRunning() && !shotTimer.hasElapsed(shotDelaySec)) {
            return; // Cooldown not done; skip creating new projectile
        }

        // Check fuel amount
        int fuelToShoot = computeFuelToShoot(available);

        // Index fuel
        for (int i = 0; i < fuelToShoot; i++) {
            intakeSimulation.obtainGamePieceFromIntake();
        }

        // Shoot fuel
        double step = (fuelToShoot > 1) ? (rightMostFuelPositionOffset - leftMostFuelPositionOffset) / (fuelToShoot - 1) : 0.0;

        for (int i = 0; i < fuelToShoot; i++) {
            double offset = (fuelToShoot == 1) ? 0.0 : leftMostFuelPositionOffset + i * step;

            createFuelProjectile(
                robotPose,
                fieldRelativeVelocity,
                hoodAngleRad,
                flywheelsSpeedRadPerSec,
                new Transform2d(0.0, offset, Rotation2d.kZero),
                onScore);
        }

        // Restart cooldown timer after firing
        shotTimer.restart();
    }
}