package org.frogforce503.lib.vision.objectdetection;

/**
 * Record representing an object detected by an ObjectDetectionIO.
 * 
 * @param classId The ID of the class of the object detected.
 * @param pitch the pitch of the center of the camera frame to the center of the object in degrees (aka ty). Pitch is positive when you tilt down.
 * @param yaw the yaw of the center of the camera frame to the center of the object in degrees (aka tx). Yaw is positive when you turn left.
 * @param area The percent (0 to 100) of the camera frame that the object occupies.
 * @param confidence The confidence score of the detection, where 0 is no confidence (bad) and 1 is maximum confidence (good)
 */
public record TrackedObject (
    int classId, //Corresponds to type of object detected

    double pitch, //ty
    double yaw, //tx

    double area,
    double confidence // Confidence score of the detection.
) {
    public TrackedObject() {
        this(-1, 0, 0, 0, 0);
    }

    /**
     * Returns whether the object is a real tracked target.
     * @return if the class ID is not -1, then it is a real tracked target
     */
    public boolean isReal() {
        return classId != -1;
    }
};