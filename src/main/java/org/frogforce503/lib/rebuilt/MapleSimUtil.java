package org.frogforce503.lib.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class MapleSimUtil {
    // Arena Constants
    private static final Translation2d blueDepotBottomLeftCorner =
        FieldConstants.Depot.blue.getCenter().getTranslation()
            .plus(
                new Translation2d(
                    -FieldConstants.Depot.blue.getXWidth() / 2,
                    FieldConstants.Depot.blue.getYWidth() / 2));

    private static final Translation2d redDepotBottomLeftCorner =
        FieldConstants.Depot.red.getCenter().getTranslation()
            .plus(
                new Translation2d(
                    FieldConstants.Depot.red.getXWidth() / 2,
                    -FieldConstants.Depot.red.getYWidth() / 2));

    private static final double fuelDiameter = Units.inchesToMeters(5.91);
    
    // Intake Constants
    private static final Distance intakeWidth = Inches.of(25.5);
    private static final Distance intakeLengthExtended = Inches.of(9.5);
    private static final int fuelCapacity = 30;

    // Hopper Constants
    private static final int cols = 5;
    private static final int rows = 3;
    private static final int perLayer = cols * rows;
    private static final double fuelToFuelOffset = Units.inchesToMeters(4);
    private static final Transform3d robotToHopperOffset = new Transform3d(new Translation3d(0, 0, Units.inchesToMeters(9)), Rotation3d.kZero);

    // Shoot Constants
    private static final Translation3d shotTolerance = new Translation3d(0.5, 0.5, 0.5);

    // Bump Constants
    private static final double maxLinearSpeedOverBumpMetersPerSec = DriveConstants.maxLinearSpeed / 5;

    private MapleSimUtil() {}
    
    public static void initializeArena() {
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false)); // Allow MapleSim to cross over bump
    }

    public static void resetArena() {
        SimulatedArena.getInstance().resetFieldForAuto();

        // Add depot fuel
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 6; y++) {
                Translation2d fuelPosition = Translation2d.kZero;

                if (FieldConstants.isRed()) {
                    fuelPosition =
                        redDepotBottomLeftCorner
                            .plus(new Translation2d(-fuelDiameter / 2, fuelDiameter + Units.inchesToMeters(0.5))) // bottom left corner to bottom left fuel offset
                            .plus(new Translation2d(-fuelDiameter * x, fuelDiameter * y));
                } else {
                    fuelPosition =
                        blueDepotBottomLeftCorner
                            .plus(new Translation2d(fuelDiameter / 2, -(fuelDiameter + Units.inchesToMeters(0.5)))) // bottom left corner to bottom left fuel offset
                            .plus(new Translation2d(fuelDiameter * x, -fuelDiameter * y));
                }

                SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(fuelPosition));
            }
        }
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

    public static Translation3d[] visualizeFuelInHopper(Pose3d robotPose, int numFuelInRobot) {
        Translation3d[] balls = new Translation3d[numFuelInRobot];

        for (int i = 0; i < numFuelInRobot; i++) {
            int layer = i / perLayer;
            int grid = i % perLayer;

            double x = (grid % cols - (cols - 1) / 2.0) * fuelToFuelOffset;
            double y = (grid / cols - (rows - 1) / 2.0) * fuelToFuelOffset;
            double z = layer * fuelToFuelOffset * 1.25; // slightly taller spacing for visibility

            balls[i] =
                robotPose
                    .plus(robotToHopperOffset)
                    .plus(new Transform3d(new Translation3d(x, y, z), Rotation3d.kZero))
                    .getTranslation();
        }

        return balls;
    }

    public static void createFuelProjectile(
        Pose2d pose,
        ChassisSpeeds robotFieldRelativeVelocity,
        Rotation2d turretFieldRelativeAngle,
        double hoodAngleRad,
        double flywheelsSpeedRadPerSec
    ) {
        GamePieceProjectile fuel =
            new RebuiltFuelOnFly(
                pose
                    .plus(GeomUtil.toTransform2d(TurretConstants.robotToTurret))
                    .plus(GeomUtil.toTransform2d(HoodConstants.turretToHood))
                    .getTranslation(),
                Translation2d.kZero,
                robotFieldRelativeVelocity,
                turretFieldRelativeAngle,
                Pose3d.kZero
                    .plus(TurretConstants.robotToTurret)
                    .plus(HoodConstants.turretToHood)
                    .plus(new Transform3d(new Translation3d(0, 0, Units.inchesToMeters(4)), Rotation3d.kZero)) // offset = 4 inches
                    .getMeasureZ(),
                MetersPerSecond.of(flywheelsSpeedRadPerSec * FlywheelsConstants.kSimRadiusMeters),
                Radians.of(Units.degreesToRadians(90) - hoodAngleRad)); // 0 deg hood = 90 deg shot angle (since shots have to go up) & vice versa

        fuel
            .withTargetPosition(() -> FieldConstants.getShotTarget(pose))
            .withTargetTolerance(shotTolerance)
            .withProjectileTrajectoryDisplayCallBack(
                pose3ds -> Logger.recordOutput("GameViz/SuccessfulFuelShot", pose3ds.toArray(Pose3d[]::new)),
                pose3ds -> Logger.recordOutput("GameViz/UnsucessfulFuelShot", pose3ds.toArray(Pose3d[]::new))
            );

        SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }
    
    /** Applies max velocity to bumps instead of blocking them out like MapleSim */
    public static ChassisSpeeds limitVelocityOverBumps(Translation2d robotTranslation, ChassisSpeeds robotVelocity) {
        double linearSpeed =
            Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

        boolean inBump = FieldConstants.Bump.contains(robotTranslation);

        if (inBump && linearSpeed > maxLinearSpeedOverBumpMetersPerSec) {
            double scalar = maxLinearSpeedOverBumpMetersPerSec / linearSpeed;

            return new ChassisSpeeds(
                robotVelocity.vxMetersPerSecond * scalar,
                robotVelocity.vyMetersPerSecond * scalar,
                robotVelocity.omegaRadiansPerSecond);
            
        }

        return robotVelocity;
    }
}