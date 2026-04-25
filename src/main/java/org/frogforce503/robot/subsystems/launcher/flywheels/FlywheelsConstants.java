package org.frogforce503.robot.subsystems.launcher.flywheels;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class FlywheelsConstants {
    // Hardware / Configuration
    public static final int leaderId = 9;
    public static final int followerId = 8;

    public static final double motorMechanismRatio = 0.75; // from CAD

    public static final boolean leaderInverted = true;
    public static final boolean followerInverted = true;

    public static final int statorCurrentLimit = 80;
    
    public static final PIDConfig kPID = new PIDConfig(0.005, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 0, 0.014, 0);
    public static final double kRateLimit = Units.rotationsPerMinuteToRadiansPerSecond(5000);

    public static final double kSimRadiusMeters = Units.inchesToMeters(1.5);

    // Setpoints
    public static final double tolerance = Units.rotationsPerMinuteToRadiansPerSecond(50.0);

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static final double IDLE = Units.rotationsPerMinuteToRadiansPerSecond(1500);
    public static final double BATTER = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static final double TRENCH = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static final double TOWER = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static final double EJECT = Units.rotationsPerMinuteToRadiansPerSecond(100);
}
