package org.frogforce503.robot.constants.hardware.subsystem_config;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public record IntakePivotConfig(
    int id,
    double mechanismRatio,

    boolean inverted,
    int statorCurrentLimit,
    double zeroOffset,

    PIDConfig kPID,
    FFConfig kFF,
    Constraints kConstraints,

    double minAngle,
    double maxAngle) {}