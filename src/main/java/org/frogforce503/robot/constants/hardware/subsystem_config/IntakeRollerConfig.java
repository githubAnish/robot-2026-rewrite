package org.frogforce503.robot.constants.hardware.subsystem_config;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

public record IntakeRollerConfig(
    int id,
    double mechanismRatio,

    boolean inverted,
    int statorCurrentLimit,
    
    PIDConfig kPID,
    FFConfig kFF) {}