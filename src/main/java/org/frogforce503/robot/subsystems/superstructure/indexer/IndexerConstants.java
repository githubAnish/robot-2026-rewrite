package org.frogforce503.robot.subsystems.superstructure.indexer;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class IndexerConstants {
    // Hardware / Configuration
    public static final int id = 4;
    public static final double mechanismRatio = 3.27272727; // from CAD

    public static final boolean inverted = true;
    public static final int statorCurrentLimit = 80;
    
    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();

    // Setpoints
    public static final double kTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0); // TODO may change based on real robot

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);

    public static final double INTAKE = Units.rotationsPerMinuteToRadiansPerSecond(2000);

    public static final double SHOOT = Units.rotationsPerMinuteToRadiansPerSecond(4000);

    public static final double EJECT = Units.rotationsPerMinuteToRadiansPerSecond(-2000);
}