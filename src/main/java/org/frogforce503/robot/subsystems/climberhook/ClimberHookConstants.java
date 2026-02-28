package org.frogforce503.robot.subsystems.climberhook;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class ClimberHookConstants {
    // Hardware / Configuration
    public static final int id = 11;
    public static final double mechanismRatio = 1;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(2);

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;

    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(0, 0);
    
    public static final double minHeight = Units.inchesToMeters(0);
    public static final double maxHeight = Units.inchesToMeters(0);

    public static final int climberLimitSwitchId = 1;

    // Setpoints
    public static final double kTolerance = Units.inchesToMeters(0.5);

    public static final double START = minHeight;

    public static final double ALGAE_PLUCK_LOW = START;
    public static final double ALGAE_PLUCK_HIGH = START;
}