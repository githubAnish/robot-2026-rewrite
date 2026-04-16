package org.frogforce503.robot.subsystems.vision.io.apriltagdetection;

import java.util.Set;

import org.frogforce503.lib.vision.apriltagdetection.PoseObservation;
import org.frogforce503.lib.vision.apriltagdetection.PoseObservationType;
import org.frogforce503.lib.vision.apriltagdetection.TrackedAprilTag;
import org.frogforce503.robot.subsystems.vision.io.VisionIO;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagIO extends VisionIO {
    @AutoLog
    class AprilTagInputs {
        public boolean persistingOldResults = false;
        public boolean connected = false;
        public boolean hasTargets = false;
        public TrackedAprilTag[] trackedAprilTags = new TrackedAprilTag[0];
    }

    void updateInputs(AprilTagInputs inputs);

    PoseObservation estimateRobotPose();

    void setPoseObservationType(PoseObservationType poseObservationType);

    default void setSecondaryPoseObservationType(PoseObservationType poseObservationType) {}

    default void setRobotPose(Pose2d pose) {}

    default void setIgnoredAprilTags(Set<Integer> ids) {}
}