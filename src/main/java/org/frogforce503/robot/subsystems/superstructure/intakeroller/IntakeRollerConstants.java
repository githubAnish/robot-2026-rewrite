package org.frogforce503.robot.subsystems.superstructure.intakeroller;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.util.Units;

public class IntakeRollerConstants {
    // Hardware / Configuration
    public static final int motorId = 3;

    public static final double motorMechanismRatio = 1.78; // from CAD

    public static final boolean motorInverted = true;
    
    public static final int statorCurrentLimit = 60;
    
    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();

    // Setpoints
    public static final double kTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0); // TODO may change based on real robot

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);

    public static final double INTAKE = Units.rotationsPerMinuteToRadiansPerSecond(2000);

    public static final double EJECT = Units.rotationsPerMinuteToRadiansPerSecond(-2000);
}