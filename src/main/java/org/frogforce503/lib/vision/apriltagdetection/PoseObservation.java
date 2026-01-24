package org.frogforce503.lib.vision.apriltagdetection;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Record representing a pose observation of the robot in 3D space made by an AprilTagIO.
 * 
 * @param timestamp The timestamp of the observation in seconds.
 * @param robotPose The pose of the robot in 3D space.
 * @param poseObservationType The type of pose observation (e.g., single tag, multi-tag, etc.).
 * @param usedAprilTags The AprilTags used in this observation.
 */
public record PoseObservation (
    double timestamp,
    Pose3d robotPose,
    PoseObservationType poseObservationType,
    TrackedAprilTag[] usedAprilTags
) {
    public PoseObservation() {
        this(0.0, new Pose3d(), PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR, new TrackedAprilTag[0]);
    }

    /**
     * Returns whether the pose observation is real.
     * 
     * @return True if the pose observation uses any tags; otherwise, false.
     */
    public boolean isReal() {
        return usedAprilTags.length > 0;
    }
};