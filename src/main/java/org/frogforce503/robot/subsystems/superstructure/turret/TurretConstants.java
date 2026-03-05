package org.frogforce503.robot.subsystems.superstructure.turret;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
    // Hardware / Configuration
    public static final int id = 4;
    public static final double mechanismRatio = 100; // currently 6328 turret gear ratio

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;
    public static final double relativeEncoderZeroOffset;
    public static final double absoluteEncoderZeroOffset;

    public static final PIDConfig kPID = new PIDConfig(2, 0, 0); // some basic pid value
    public static final FFConfig kFF = new FFConfig(0, 0, 2, 0);
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(2700));
    
    public static final double minAngle = Units.degreesToRadians(-210.0); // Assume 6328 params here
    public static final double maxAngle = Units.degreesToRadians(210.0); // Assume 6328 params here

    public static final Transform3d robotToTurret =
        new Transform3d(
            Units.inchesToMeters(-6),
            Units.inchesToMeters(-8.5),
            Units.inchesToMeters(13.91),
            Rotation3d.kZero);
        
    public static final Transform3d turretToCamera =
        new Transform3d(
            Units.inchesToMeters(5.493439),
            Units.inchesToMeters(2.075000),
            Units.inchesToMeters(6.244572),
            new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0));

    static {
        switch (Constants.getRobot()) {
            case SimBot -> {
                relativeEncoderZeroOffset = Units.degreesToRadians(90);
                absoluteEncoderZeroOffset = 0.0;
            }
            default -> { // Use comp bot params
                relativeEncoderZeroOffset = Units.degreesToRadians(90);
                absoluteEncoderZeroOffset = 0.1788;
            }
        }
    }

    // Setpoints
    public static final double kFixedTolerance = Units.degreesToRadians(0.5); // TODO Turret position has to be extremely accurate for consistent shot
    public static final double kShootOnMoveTolerance = Units.degreesToRadians(5); // TODO Turret position has to be extremely accurate for consistent shot

    public static final double START = Units.degreesToRadians(0);
    public static final double CLIMB = Units.degreesToRadians(180); // robot-relative, faces backwards

    public static final Rotation2d BATTER_FIELD_RELATIVE = Rotation2d.fromDegrees(0);
    public static final Rotation2d TRENCH_FIELD_RELATIVE = Rotation2d.fromDegrees(0);
    public static final Rotation2d DEPOT_FIELD_RELATIVE = Rotation2d.fromDegrees(0);
}