package org.frogforce503.robot.subsystems.vision.io.apriltagdetection;

import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {
    @AutoLog
    class AprilTagIOInputs {
        
    }

    default void updateInputs(AprilTagIOInputs inputs) {}
}