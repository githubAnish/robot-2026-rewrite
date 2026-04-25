package org.frogforce503.lib.rebuilt.sim.maplesim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator;
import org.frogforce503.robot.subsystems.launcher.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class MapleSimUtil {
    private static final boolean isDebugging = false;

    // Arena Constants
    private static final FFArena2026Rebuilt arena = new FFArena2026Rebuilt(false);
    
    // Intake Constants
    private static final Distance intakeWidth = Inches.of(25.5);
    private static final Distance intakeLengthExtended = Inches.of(9.5);
    public static final int fuelCapacity = 70;

    // Shoot Constants
    private static final Translation3d shotTolerance = new Translation3d(0.2, 0.2, 0.2);
    private static final Transform2d initialFuelPositionOffset = new Transform2d(Units.inchesToMeters(3), 0, Rotation2d.kZero);
    private static final Transform3d initialShotHeightOffset = new Transform3d(0, 0, Units.inchesToMeters(4), Rotation3d.kZero);

    private MapleSimUtil() {}
    
    public static void initializeArena() {
        SimulatedArena.overrideInstance(arena); // Allow MapleSim to cross over bump
    }

    public static void logObstaclesInArena(Field2d field2d) {
        arena.logObstacles(field2d);
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

    public static void createFuelProjectile(
        Pose2d robotPose,
        ChassisSpeeds robotFieldRelativeVelocity,
        double hoodAngleRad,
        double flywheelsSpeedRadPerSec,
        Transform2d fuelLaunchPositionOffset,
        Runnable addScore
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
            .withTargetPosition(() -> LaunchCalculator.getShotTarget(robotPose))
            .withTargetTolerance(shotTolerance)
            .setHitTargetCallBack(addScore);

        if (isDebugging) {
            fuel.withProjectileTrajectoryDisplayCallBack(
                pose3ds -> Logger.recordOutput("GameViz/SuccessfulFuelShot", pose3ds.toArray(Pose3d[]::new)),
                pose3ds -> Logger.recordOutput("GameViz/UnsucessfulFuelShot", pose3ds.toArray(Pose3d[]::new))
            );
        }

        SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }
}