package org.frogforce503.lib.vision.apriltagdetection;

/** Record representing an AprilTag detected by an AprilTagIO. */
public record TrackedAprilTag (
    int tagID,
    double pitch, // ty
    double yaw, // tx
    double area,
    double distance,
    double ambiguity
) {
    public TrackedAprilTag() {
        this(-1, 0, 0, 0, 0, 0);
    }

    public boolean isReal() {
        return tagID != -1;
    }
};