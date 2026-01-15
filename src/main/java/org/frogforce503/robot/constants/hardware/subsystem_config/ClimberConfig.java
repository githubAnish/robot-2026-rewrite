package org.frogforce503.robot.constants.hardware.subsystem_config;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public record ClimberConfig(
    int id,
    double mechanismRatio,
    double sprocketPitchDiameter,

    boolean inverted,
    int statorCurrentLimit,

    PIDConfig kPID,
    FFConfig kFF,
    Constraints kConstraints,
    
    double minHeight,
    double maxHeight) {}