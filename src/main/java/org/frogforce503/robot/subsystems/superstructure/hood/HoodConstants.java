package org.frogforce503.robot.subsystems.superstructure.hood;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class HoodConstants {
    // Hardware / Configuration
    public static final int motorId = 7;

    public static final double motorMechanismRatio = 15.0; // from CAD
    public static final double absoluteEncoderMechanismRatio = 1.0;

    public static final boolean motorInverted = false;
    public static final boolean absoluteEncoderInverted = false;

    public static final int statorCurrentLimit = 20;

    public static final double absoluteEncoderZeroOffset = 0.651;

    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(0), Units.degreesToRadians(0));
    
    public static final double minAngle = Units.degreesToRadians(0);
    public static final double maxAngle = Units.degreesToRadians(37);

    public static final Transform3d turretToHood = new Transform3d(0.09, 0.0, 0.09, Rotation3d.kZero); // Sim turret to hood
    
    // Setpoints
    public static final double kFixedTolerance = Units.degreesToRadians(0.5);
    public static final double kShootOnMoveTolerance = Units.degreesToRadians(10);

    public static final double START = Units.degreesToRadians(0);
    public static final double CLIMB = Units.degreesToRadians(0);

    public static final double DUCK_UNDER_TRENCH = Units.degreesToRadians(0);

    public static final double BATTER = Units.degreesToRadians(0);
    public static final double TRENCH = Units.degreesToRadians(0);
    public static final double DEPOT = Units.degreesToRadians(0);
}