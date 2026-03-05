package org.frogforce503.robot.subsystems.superstructure.hood;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class HoodConstants {
    // Hardware / Configuration
    public static final int id = 7;
    public static final double mechanismRatio = 15.0;
    public static final double absoluteEncoderMechanismRatio = 1.0;

    public static final boolean motorInverted = false;
    public static final boolean absoluteEncoderInverted = false;
    public static final int statorCurrentLimit = 20;

    public static final double absoluteEncoderZeroOffset = 0.651;

    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(0, 0);
    
    public static final double minAngle = Units.degreesToRadians(0); // TODO 0 deg is when hood horizontal (ball shoots horizontally)
    public static final double maxAngle = Units.degreesToRadians(37);// TODO 90 deg is when hood vertical (ball shoots verticalally)

    public static final Transform3d turretToHood = new Transform3d(0.09, 0.0, 0.09, Rotation3d.kZero); // Sim turret to hood, can change based on real robot CAD
    
    // Setpoints
    public static final double kFixedTolerance = Units.degreesToRadians(0.5); //TODO Hood position has to be extremely accurate for consistent shot
    public static final double kShootOnMoveTolerance = Units.degreesToRadians(10); //TODO Hood position has to be extremely accurate for consistent shot

    public static final double START = Units.degreesToRadians(0);
    public static final double CLIMB = Units.degreesToRadians(0);

    public static final double DUCK_UNDER_TRENCH = Units.degreesToRadians(0);

    public static final double BATTER = Units.degreesToRadians(0);
    public static final double TRENCH = Units.degreesToRadians(0);
    public static final double DEPOT = Units.degreesToRadians(0);
}