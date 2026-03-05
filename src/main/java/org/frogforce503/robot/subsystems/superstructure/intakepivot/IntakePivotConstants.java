package org.frogforce503.robot.subsystems.superstructure.intakepivot;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
    // Hardware / Configuration
    public static final int id = 2;
    public static final double mechanismRatio = 20.0;

    public static final boolean motorInverted = false;
    public static final boolean absoluteEncoderInverted = true;
    public static final int statorCurrentLimit = 80;
    public static final double absoluteEncoderZeroOffset = 0.787;

    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(0, 0);

    public static final double minAngle = Units.degreesToRadians(-5.0);
    public static final double maxAngle = Units.degreesToRadians(90);

    // Setpoints
    public static final double kTolerance = Units.degreesToRadians(3.0);

    public static final double START = maxAngle;

    public static final double STOW = Units.degreesToRadians(89);
    public static final double INTAKE = Units.degreesToRadians(30);
    public static final double EJECT = INTAKE;
}