package org.frogforce503.lib.vision.apriltag_detection;

/**
 * Enum representing different types of pose observations for AprilTag detection.
 * This enum is used to specify the method of pose estimation based on the detection system being used (e.g., Limelight, PhotonVision).
 */
public enum PoseObservationType {
    //Limelight
    //Docs https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization 
    MEGATAG1,
    MEGATAG2,

    //PhotonVision 
    //Docs https://docs.photonvision.org/en/v2025.3.2/docs/programming/photonlib/robot-pose-estimator.html
    MULTI_TAG_PNP_ON_COPROCESSOR,
    CONSTRAINED_SOLVEPNP,
    PNP_DISTANCE_TRIG_SOLVE,
    LOWEST_AMBIGUITY,
    CLOSEST_TO_CAMERA_HEIGHT,
    CLOSEST_TO_REFERENCE_POSE,
    CLOSEST_TO_LAST_POSE,
    AVERAGE_BEST_TARGETS;
}
