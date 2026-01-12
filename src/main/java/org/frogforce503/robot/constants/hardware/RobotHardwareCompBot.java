package org.frogforce503.robot.constants.hardware;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.constants.hardware.subsystem_config.*;
import org.frogforce503.robot.constants.tuner.TunerConstantsCompBot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class RobotHardwareCompBot extends RobotHardware {
    public RobotHardwareCompBot() {
        // Create drive config
        this.driveConfig =
            new DriveConfig(
                TunerConstantsCompBot.DrivetrainConstants,
                TunerConstantsCompBot.FrontLeft,
                TunerConstantsCompBot.FrontRight,
                TunerConstantsCompBot.BackLeft,
                TunerConstantsCompBot.BackRight);

        // Create vision config
        this.visionConfig =
            new VisionConfig();
            
        // Create superstructure configs
        this.intakeRollerConfig =
            new IntakeRollerConfig(
                0,
                0,
                false,
                80,
                new PIDConfig(),
                new FFConfig());

        this.flywheelsConfig =
            new FlywheelsConfig(
                0,
                0,
                false,
                80,
                new PIDConfig(),
                new FFConfig());

        this.hoodConfig =
            new HoodConfig(
                0,
                0,
                false,
                80,
                0,
                new PIDConfig(),
                new FFConfig(),
                new Constraints(0, 0),
                Units.degreesToRadians(0), // 0 deg is when hood horizontal (ball shoots horizontally)
                Units.degreesToRadians(90)); // 90 deg is when hood vertical (ball shoots verticalally)
        
        // Create other configs
        this.ledsConfig =
            new LedsConfig(11);

        this.followerLinearPID = new PIDConfig(5.0, 0.0, 0.0);
        this.followerThetaPID = new PIDConfig(4.0, 0.0, 0.0);
    }
}