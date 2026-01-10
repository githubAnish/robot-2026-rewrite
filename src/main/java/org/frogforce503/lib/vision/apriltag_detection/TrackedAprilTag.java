package org.frogforce503.lib.vision.apriltag_detection;

/**
 * Record representing an AprilTag detected by an AprilTagIO.
 * 
 * @param tagID The fiducial ID of the AprilTag.
 * @param pitch The pitch from the center of the camera frame to the center of the AprilTag in degrees (also known as ty).
 * @param yaw The yaw from the center of the camera frame to the center of the AprilTag in degrees (also known as tx).
 * @param area The percent (0 to 100) of the camera frame that the AprilTag occupies.
 * @param distance The distance from the camera to the AprilTag in meters.
 * @param ambiguity The ambiguity of the AprilTag detection, where 0 represents no ambiguity (good) and 1 represents maximum ambiguity (bad).
 */
public record TrackedAprilTag (
    int tagID,
    double pitch, // ty
    double yaw,   // tx
    double area,
    double distance,
    double ambiguity
) {
    public TrackedAprilTag() {
        this(-1, 0, 0, 0, 0, 0);
    }

    /**
     * Returns whether the AprilTag is a real tracked target.
     * 
     * @return True if the tag ID is not -1, indicating it is a real tracked target; otherwise, false.
     */
    public boolean isReal() {
        return tagID != -1;
    }
};