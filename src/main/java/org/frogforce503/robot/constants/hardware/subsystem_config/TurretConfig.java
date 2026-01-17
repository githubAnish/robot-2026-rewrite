package org.frogforce503.robot.constants.hardware.subsystem_config;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

public record TurretConfig(
    int id,
    double mechanismRatio,

    boolean inverted,
    int statorCurrentLimit,
    double zeroOffset,

    PIDConfig kPID,
    FFConfig kFF,
    double kRateLimit,
    
    double minAngle,
    double maxAngle) {}