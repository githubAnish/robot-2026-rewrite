package org.frogforce503.robot.subsystems.intakeroller;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class IntakeRollerConstants {
    // Hardware / Configuration
    public static final int motorId = 3;

    public static final double motorMechanismRatio = 1.78; // from CAD

    public static final boolean motorInverted = true;
    
    public static final int statorCurrentLimit = 80;
    
    public static final PIDConfig kPID = new PIDConfig(0.00001, 0, 0);
    public static final FFConfig kFF = new FFConfig(0, 0, 0.0397, 0);

    // Setpoints
    public static final double tolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0);

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static final double INTAKE = Units.rotationsPerMinuteToRadiansPerSecond(2000);
    public static final double EJECT = Units.rotationsPerMinuteToRadiansPerSecond(-2000);
}