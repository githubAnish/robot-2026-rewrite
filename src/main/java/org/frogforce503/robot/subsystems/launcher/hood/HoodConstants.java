package org.frogforce503.robot.subsystems.launcher.hood;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class HoodConstants {
    // Hardware / Configuration
    public static final int motorId = 7;

    public static final double motorMechanismRatio = 1.54545174;
    public static final double absoluteEncoderMechanismRatio = 1.0;

    public static final boolean motorInverted = false;
    public static final boolean absoluteEncoderInverted = false;

    public static final int statorCurrentLimit = 80;

    public static final double absoluteEncoderZeroOffset = 0.651;

    public static final PIDConfig kPID = new PIDConfig(1, 0, 0.5);
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720));
    
    public static final double minAngle = Units.degreesToRadians(0.5); // from robot-2026 repo
    public static final double maxAngle = Units.degreesToRadians(34); // from robot-2026 repo

    public static final Transform3d robotToHood = new Transform3d(-0.27, 0.0, 0.483, Rotation3d.kZero); // Sim robot to hood
    
    // Setpoints
    public static final double fixedTolerance = Units.degreesToRadians(0.5);
    public static final double shootOnMoveTolerance = Units.degreesToRadians(10);

    public static final double START = Units.degreesToRadians(0);
    public static final double DUCK_UNDER_TRENCH = Units.degreesToRadians(0);
    public static final double BATTER = Units.degreesToRadians(0);
    public static final double TRENCH = Units.degreesToRadians(0);
    public static final double TOWER = Units.degreesToRadians(0);
    public static final double CLIMB = Units.degreesToRadians(0);
}