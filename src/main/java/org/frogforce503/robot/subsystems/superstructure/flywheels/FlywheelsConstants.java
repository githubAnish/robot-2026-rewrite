package org.frogforce503.robot.subsystems.superstructure.flywheels;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class FlywheelsConstants {
    // Hardware / Configuration
    public static final int id = 5;
    public static final double mechanismRatio = 1;

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;
    
    public static final PIDConfig kPID = new PIDConfig(0.1, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 0, 0.0224, 0);
    public static final double kRateLimit = Units.rotationsPerMinuteToRadiansPerSecond(6000);

    // Setpoints
    public static final double kTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0); // TODO Flywheel speed has to be extremely accurate for consistent shot

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);

    public static final double IDLE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(1500);

    public static final double EJECT = Units.rotationsPerMinuteToRadiansPerSecond(1500);
}
