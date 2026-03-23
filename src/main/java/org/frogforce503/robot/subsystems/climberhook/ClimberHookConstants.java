package org.frogforce503.robot.subsystems.climberhook;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ClimberHookConstants {
    // Hardware / Configuration
    public static final int motorId = 11;
    public static final int limitSwitchId = 1;

    public static final double mechanismRatio = 27.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.89); // from JVN calc

    public static final boolean motorInverted = false;
    
    public static final int statorCurrentLimit = 80;

    public static final PIDConfig kPID = new PIDConfig(10, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 1.794, 15, 0);
    public static final Constraints kConstraints = new Constraints(Units.inchesToMeters(45), Units.inchesToMeters(90));
    
    public static final double minHeight = Units.inchesToMeters(-10);
    public static final double maxHeight = Units.inchesToMeters(10);

    // Setpoints
    public static final double kTolerance = Units.inchesToMeters(1);

    public static final double START = Units.inchesToMeters(0);
}