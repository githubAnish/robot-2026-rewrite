package org.frogforce503.robot.subsystems.superstructure.turret;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class TurretConstants {
    // Hardware / Configuration
    public static final int motorId = 6;

    public static final double motorMechanismRatio = 250.0;
    public static final double absoluteEncoderMechanismRatio = 1.0;
    
    public static final boolean motorInverted = true;
    public static final boolean absoluteEncoderInverted = true;

    public static final int statorCurrentLimit = 35;

    public static final double relativeEncoderZeroOffsetRad = RobotBase.isSimulation() ? 0.0 : Units.degreesToRadians(90);
    public static final double absoluteEncoderZeroOffset = 0.1788;

    public static final PIDConfig kPID = new PIDConfig(8, 0, 0.2);
    public static final FFConfig kFF = new FFConfig(0, 0, 8, 0);
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720));
    
    public static final double minAngle = Units.degreesToRadians(-132.0); // measured from 0 deg (which is when turret faces opposite the intake)
    public static final double maxAngle = Units.degreesToRadians(200.0); // measured from 0 deg (which is when turret faces opposite the intake)

    public static final Transform3d robotToTurret =
        new Transform3d(
            Units.inchesToMeters(-6),
            Units.inchesToMeters(-8.5),
            Units.inchesToMeters(13.91),
            Rotation3d.kZero);

    // Setpoints
    public static final double kFixedTolerance = Units.degreesToRadians(0.5);
    public static final double kShootOnMoveTolerance = Units.degreesToRadians(5);

    public static final double START = Units.degreesToRadians(0);
    public static final double CLIMB = Units.degreesToRadians(180); // robot-relative, faces backwards

    public static final Rotation2d BATTER_FIELD_RELATIVE = Rotation2d.fromDegrees(0);
    public static final Rotation2d TRENCH_FIELD_RELATIVE = Rotation2d.fromDegrees(0);
    public static final Rotation2d DEPOT_FIELD_RELATIVE = Rotation2d.fromDegrees(0);
}