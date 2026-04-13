package org.frogforce503.robot.subsystems.climber;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    // Hardware / Configuration
    public static final int motorId = 10;
    public static final int limitSwitchId = 9;

    public static final double mechanismRatio = 45.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.0);

    public static final boolean motorInverted = true;
    
    public static final int statorCurrentLimit = 80;

    public static final PIDConfig kPID = new PIDConfig(10, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 1.794, 15, 0);
    public static final Constraints kConstraints = new Constraints(Units.inchesToMeters(45), Units.inchesToMeters(90));
    
    public static final double minHeight = Units.inchesToMeters(0);
    public static final double maxHeight = Units.inchesToMeters(13.5);

    // Setpoints
    public static final double tolerance = Units.inchesToMeters(1);

    public static final double START = minHeight;
    public static final double RAISE = Units.inchesToMeters(13.4);
    public static final double LOWER = Units.inchesToMeters(10.4);
}