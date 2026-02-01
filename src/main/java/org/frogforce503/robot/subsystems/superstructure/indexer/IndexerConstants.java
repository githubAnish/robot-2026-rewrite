package org.frogforce503.robot.subsystems.superstructure.indexer;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class IndexerConstants {
    // Hardware / Configuration
    public static final int id = 2;
    public static final double mechanismRatio = 1;

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;
    
    public static final PIDConfig kPID = new PIDConfig(0.00001, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 0, 0.0225, 0);

    // Setpoints
    public static final double kTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0); // TODO may change based on real robot

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);

    public static final double SLOW_MIX = Units.rotationsPerMinuteToRadiansPerSecond(500);

    public static final double INTAKE = Units.rotationsPerMinuteToRadiansPerSecond(2000);

    public static final double MIN_FREE_SPEED =
        Units.rotationsPerMinuteToRadiansPerSecond(200); // tune on robot

    public static final double COMPRESSION_CURRENT = 30.0; // amps, tune

}
