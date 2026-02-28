package org.frogforce503.lib.vision.objectdetection;

import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Record representing an object detected by an ObjectDetectionIO.
 * 
 * @param classId The ID of the class of the object detected.
 * @param pitch the pitch of the center of the camera frame to the center of the object in degrees (aka ty). Pitch is positive when you tilt down.
 * @param yaw the yaw of the center of the camera frame to the center of the object in degrees (aka tx). Yaw is positive when you turn left.
 * @param area The percent (0 to 100) of the camera frame that the object occupies.
 * @param confidence The confidence score of the detection, where 0 is no confidence (bad) and 1 is maximum confidence (good)
 * @param boundingBoxCornersX The x-coords of corners; ordered bottom left, bottom right, top right, top left; x is postive to right
 * @param boundingBoxCornersY The y-coords of corners; ordered bottom left, bottom right, top right, top left; y is postive downwards
 */
public record TrackedObject (
    int classId, //Corresponds to type of object detected

    double pitch, //ty
    double yaw, //tx

    double area,
    double confidence, // Confidence score of the detection.

    /* list of the n corners in image space (origin top left, x right, y down), in no
     * particular order, detected for this target.
     *
     * ⟶ +X  3 ----- 2
     * |      |       |
     * V      |       |
     * +Y     0 ----- 1
    */
    double[] boundingBoxCornersX, // The x-coordinates of the corners of the bounding box around the detected object, in pixels
    double[] boundingBoxCornersY // The y-coordinates of the corners of the bounding box around the detected object, in pixels
) {
    public TrackedObject() {
        this(-1, 0, 0, 0, 0, new double[4], new double[4]);
    }

    /**
     * Returns whether the object is a real tracked target.
     * @return if the class ID is not -1, then it is a real tracked target
     */
    public boolean isReal() {
        return classId != -1;
    }

    /** 
     * Creates a TrackedObject using PhotonLib's PhotonTargetTarget from a real camera
     */
    public static TrackedObject fromPhotonVisionTarget(PhotonTrackedTarget object) {
        return new TrackedObject(
            object.getDetectedObjectClassID(),
            object.getPitch(),
            object.getYaw(),
            object.getArea(),
            (double) object.getDetectedObjectConfidence(),
            object.detectedCorners.stream()
                .mapToDouble(corner -> corner.x)
                .toArray(),
            object.detectedCorners.stream()
                .mapToDouble(corner -> corner.y)
                .toArray()
        );
    }
     /**
     * Creates a TrackedObject using PhotonLib's PhotonTargetTarget from simulation
      */
     
    public static TrackedObject fromPhotonSimTarget(PhotonTrackedTarget object) {
        return new TrackedObject(
            object.getFiducialId(),
            object.getPitch(),
            object.getYaw(),
            object.getArea(),
            1.0,
            object.detectedCorners.stream()
                .mapToDouble(corner -> corner.x)
                .toArray(),
            object.detectedCorners.stream()
                .mapToDouble(corner -> corner.y)
                .toArray()
        );
    }
};