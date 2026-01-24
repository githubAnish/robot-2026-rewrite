package org.frogforce503.robot.subsystems.vision.apriltagdetection;

import java.util.Set;

import org.frogforce503.lib.vision.apriltagdetection.PoseObservation;
import org.frogforce503.lib.vision.apriltagdetection.PoseObservationType;
import org.frogforce503.lib.vision.apriltagdetection.TrackedAprilTag;
import org.frogforce503.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Interface to define IO functions for an AprilTag detection solution.
 */
public interface AprilTagIO extends VisionIO {
    
    /**
     * Class to hold the inputs for AprilTag detection.
     * 
     * @param persistingOldResults Boolean indicating if old results are being persisted.
     * @param connected Boolean indicating if the camera is connected.
     * @param hasTargets Boolean indicating if there are targets detected.
     * @param trackedAprilTags Array of tracked AprilTags.
     */
    @AutoLog
    class AprilTagInputs {
        public boolean persistingOldResults = false;
        public boolean connected = false;
        public boolean hasTargets = false;

        public TrackedAprilTag[] trackedAprilTags = new TrackedAprilTag[0];
    }

    /**
     * Updates the AprilTag inputs object with the latest data from an IO.
     * 
     * @param inputs The AprilTag detection inputs to be updated.
     */
    void updateInputs(AprilTagInputs inputs);

    /**
     * Estimates the robot's pose based on the latest AprilTag detections.
     * Make sure to update the inputs before calling this method.
     * 
     * @return The pose observation containing the timestamp, estimated robot pose, the april tags used, and the pose observation type.
     */
    PoseObservation estimateRobotPose();

    /**
     * Sets the primary pose observation type for pose estimation.
     * 
     * @param poseObservationType The type of pose observation to use for pose estimation.
     */
    void setPoseObservationType(PoseObservationType poseObservationType);

    /**
     * Sets a multi-tag fallback pose observation type if the primary pose observation type requires multiple tags.
     * 
     * @param poseObservationType The type of pose observation to use for pose estimation. Must NOT require multiple tags.
     */
    default void setSecondaryPoseObservationType(PoseObservationType poseObservationType) {}

    /**
     * Sets the robot's pose as a reference for pose estimation.
     * 
     * @param pose The robot's pose in meters and radians.
     */
    default void setRobotPose(Pose2d pose) {}

    /**
     * Ignores AprilTags that should not be used for pose estimation.
     * 
     * @param ids Integer set of AprilTag IDs to ignore.
     */
    default void setIgnoredAprilTags(Set<Integer> ids) {}
}