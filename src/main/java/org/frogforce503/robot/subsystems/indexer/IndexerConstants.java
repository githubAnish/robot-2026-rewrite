package org.frogforce503.robot.subsystems.indexer;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class IndexerConstants {
    // Hardware / Configuration
    public static final int motorId = 4;

    public static final double motorMechanismRatio = 3.27272727; // from CAD

    public static final boolean motorInverted = true;
    
    public static final int statorCurrentLimit = 80;
    
    public static final PIDConfig kPID = new PIDConfig(0.00001, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 0, 0.073, 0);

    // Setpoints
    public static final double tolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0);

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static final double SHOOT = Units.rotationsPerMinuteToRadiansPerSecond(4000);
    public static final double EJECT = Units.rotationsPerMinuteToRadiansPerSecond(-2000);
}