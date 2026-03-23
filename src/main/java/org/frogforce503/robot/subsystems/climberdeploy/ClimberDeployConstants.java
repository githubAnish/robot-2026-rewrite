package org.frogforce503.robot.subsystems.climberdeploy;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ClimberDeployConstants {
    // Hardware / Configuration
    public static final int motorId = 10;
    public static final int limitSwitchId = 0;

    public static final double motorMechanismRatio = 36.0;
    public static final double absoluteEncoderMechanismRatio = 1.0;

    public static final boolean motorInverted = false;
    public static final boolean absoluteEncoderInverted = false;

    public static final int statorCurrentLimit = 80;

    public static final double absoluteEncoderZeroOffset = 0.0;

    public static final PIDConfig kPID = new PIDConfig(1, 0, 0);
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720));

    public static final double minAngle = Units.degreesToRadians(0);
    public static final double maxAngle = Units.degreesToRadians(180);

    // Setpoints
    public static final double kTolerance = Units.degreesToRadians(3.0);

    public static final double START = maxAngle;
    public static final double CLIMB = minAngle;
}