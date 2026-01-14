package org.frogforce503.robot.constants.hardware;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.constants.hardware.subsystem_config.*;

import lombok.Getter;

public abstract class RobotHardware {
    // Subsystem Configs
    @Getter protected DriveConfig driveConfig;
    @Getter protected VisionConfig visionConfig;

    @Getter protected IntakePivotConfig intakePivotConfig;
    @Getter protected IntakeRollerConfig intakeRollerConfig;
    @Getter protected IndexerConfig indexerConfig;
    @Getter protected FeederConfig feederConfig;
    @Getter protected TurretConfig turretConfig;
    @Getter protected FlywheelsConfig flywheelsConfig;
    @Getter protected HoodConfig hoodConfig;

    @Getter protected LedsConfig ledsConfig;

    // Other
    @Getter protected PIDConfig followerLinearPID;
    @Getter protected PIDConfig followerThetaPID;
}