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
        this.intakePivotConfig =
            new IntakePivotConfig(
                0,
                1,
                false,
                80,
                0.0,
                new PIDConfig(),
                new FFConfig(),
                new Constraints(0, 0),
                Units.degreesToRadians(0), // TODO Can't say main linkage bar is 0 deg, but can change once block CAD / real CAD comes out
                Units.degreesToRadians(90)); // TODO basically when the 4-bar intake is stowed, the main linkage bar is at 90 deg

        this.intakeRollerConfig =
            new IntakeRollerConfig(
                1,
                1,
                false,
                80,
                new PIDConfig(),
                new FFConfig());

        this.indexerConfig =
            new IndexerConfig(
                2,
                1,
                false,
                80,
                new PIDConfig(),
                new FFConfig());

        this.feederConfig =
            new FeederConfig(
                3,
                1,
                false,
                80,
                new PIDConfig(),
                new FFConfig());

        this.turretConfig =
            new TurretConfig(
                4,
                100, // currently 6328 turret gear ratio
                false,
                80,
                0.0,
                new PIDConfig(1, 0, 0), // some basic pid value
                new FFConfig(),
                Units.degreesToRadians(360), // assume 360 deg/sec for now
                Units.degreesToRadians(-210), // TODO assume 6328 min angle for now
                Units.degreesToRadians(210)); // TODO assume 6328 min angle for now

        this.flywheelsConfig =
            new FlywheelsConfig(
                5,
                1,
                false,
                80,
                new PIDConfig(),
                new FFConfig(),
                Units.rotationsPerMinuteToRadiansPerSecond(10000)); // Maybe a high rate limit, it is 10000 rpm/sec^2, so generally will speed up flywheel in 0.5 sec if max speed is 5000 rpm

        this.hoodConfig =
            new HoodConfig(
                6,
                1,
                false,
                80,
                new PIDConfig(),
                new FFConfig(),
                new Constraints(0, 0),
                Units.degreesToRadians(0), // TODO 0 deg is when hood horizontal (ball shoots horizontally)
                Units.degreesToRadians(90)); // TODO 90 deg is when hood vertical (ball shoots verticalally)
        
        // Create other configs
        this.climberConfig =
            new ClimberConfig(
                7,
                1,
                Units.inchesToMeters(2),
                false,
                80,
                new PIDConfig(),
                new FFConfig(),
                new Constraints(0, 0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0));

        this.sensorConfig =
            new SensorConfig(0);

        this.ledsConfig =
            new LedsConfig(11);

        this.followerLinearPID = new PIDConfig(5.0, 0.0, 0.0);
        this.followerThetaPID = new PIDConfig(4.0, 0.0, 0.0);
    }
}