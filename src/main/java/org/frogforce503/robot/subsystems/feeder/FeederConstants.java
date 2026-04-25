package org.frogforce503.robot.subsystems.feeder;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class FeederConstants {
    // Hardware / Configuration
    public static final int motorId = 5;

    public static final double motorMechanismRatio = 1.0; // from CAD

    public static final boolean motorInverted = false;

    public static final int statorCurrentLimit = 80;
    
    public static final PIDConfig kPID = new PIDConfig(0.001, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 0, 0.0225, 0);

    // Setpoints
    public static final double tolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0);

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static final double IDLE = Units.rotationsPerMinuteToRadiansPerSecond(1500);
    public static final double SHOOT = Units.rotationsPerMinuteToRadiansPerSecond(2000);
    public static final double EJECT_FROM_INTAKE = Units.rotationsPerMinuteToRadiansPerSecond(-100);
}