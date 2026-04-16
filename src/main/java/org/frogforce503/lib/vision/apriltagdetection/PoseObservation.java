package org.frogforce503.lib.vision.apriltagdetection;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;

/** Represents a pose observation of the robot in 3D space made by an AprilTagIO. */
public record PoseObservation (
    double timestamp,
    Pose3d robotPose,
    PoseObservationType poseObservationType,
    TrackedAprilTag[] usedAprilTags
) {
    public static PoseObservation kZero =
        new PoseObservation(
            0.0,
            Pose3d.kZero,
            PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR,
            new TrackedAprilTag[0]);

    public PoseObservation(EstimatedRobotPose estimatedRobotPose, PoseObservationType poseObservationType) {
        this(
            estimatedRobotPose.timestampSeconds,
            estimatedRobotPose.estimatedPose,
            poseObservationType, // Use primary if multiple tags are used, otherwise use secondary
            estimatedRobotPose.targetsUsed
                .stream()
                .map(
                    tag ->
                        new TrackedAprilTag(
                            tag.getFiducialId(),
                            tag.getPitch(),
                            tag.getYaw(),
                            tag.getArea(),
                            tag.getBestCameraToTarget().getTranslation().getNorm(),
                            tag.getPoseAmbiguity())
                )
                .toArray(TrackedAprilTag[]::new)
        );
    }
    
    public boolean isReal() {
        return usedAprilTags.length > 0;
    }
};