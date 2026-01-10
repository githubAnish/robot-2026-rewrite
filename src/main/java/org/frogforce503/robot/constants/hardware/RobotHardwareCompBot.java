package org.frogforce503.robot.constants.hardware;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.constants.hardware.subsystem_config.*;
import org.frogforce503.robot.constants.tuner.TunerConstantsCompBot;

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
        
        // Create other configs
        this.ledsConfig =
            new LedsConfig(11);

        this.followerLinearPID = new PIDConfig(5.0, 0.0, 0.0);
        this.followerThetaPID = new PIDConfig(4.0, 0.0, 0.0);
    }
}