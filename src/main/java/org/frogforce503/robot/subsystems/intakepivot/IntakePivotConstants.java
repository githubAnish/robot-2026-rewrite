package org.frogforce503.robot.subsystems.intakepivot;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
    // Hardware / Configuration
    public static final int motorId = 2;

    public static final double motorMechanismRatio = 20.0; // from comp bot
    public static final double absoluteEncoderMechanismRatio = 1.0;

    public static final boolean motorInverted = false;
    public static final boolean absoluteEncoderInverted = true;

    public static final int statorCurrentLimit = 80;

    public static final double absoluteEncoderZeroOffset = 0.787;

    public static final PIDConfig kPID = new PIDConfig(2.3, 0, 0.2);
    public static final FFConfig kFF = new FFConfig(0, 3.596915, 0.65, 0);
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(720), Units.degreesToRadians(1440));
    public static final Constraints kSlowConstraints = new Constraints(Units.degreesToRadians(45), Units.degreesToRadians(90));

    public static final double minAngle = Units.degreesToRadians(-5.0);
    public static final double maxAngle = Units.degreesToRadians(110);

    // Setpoints
    public static final double tolerance = Units.degreesToRadians(3.0);

    public static final double START = maxAngle;

    public static final double STOW = START;
    public static final double INTAKE = Units.degreesToRadians(-5);
    public static final double SHOOT = Units.degreesToRadians(85);
}