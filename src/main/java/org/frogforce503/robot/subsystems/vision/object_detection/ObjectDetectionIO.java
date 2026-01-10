package org.frogforce503.robot.subsystems.vision.object_detection;

import org.frogforce503.lib.vision.object_detection.ObjectSortingMode;
import org.frogforce503.lib.vision.object_detection.TrackedObject;
import org.frogforce503.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface to define the I/O functions of an object detection solution.
 */
public interface ObjectDetectionIO extends VisionIO {
    
    /**
     * Class to hold the inputs for object detection.
     * 
     * @param persistingOldResults Boolean indicating if old results are being persisted.
     * @param connected A boolean indicating if the camera is connected.
     * @param hasTargets A boolean indicating if targets are detected.
     * @param trackedObjects An array of tracked objects detected by the camera.
     */
    @AutoLog
    class ObjectDetectionInputs {
        public boolean persistingOldResults = false;
        public boolean connected = false;
        public boolean hasTargets = false;

        public TrackedObject[] trackedObjects = new TrackedObject[0];
    }

    /**
     * Updates the object detection inputs object with the latest data from the I/O.
     * 
     * @param inputs The object detection inputs to be updated.
     */
    void updateInputs(ObjectDetectionInputs inputs);
    
    /**
     * Gets the sorting mode used to sort tracked objects.
     * 
     * @return The ObjectSortingMode enum representing the current sorting mode.
     */
    ObjectSortingMode getSortingMode();

    /**
     * Sets the sorting mode used to sort tracked objects.
     * 
     * @param sortingMode The ObjectSortingMode enum representing the desired sorting mode.
     */
    void setSortingMode(ObjectSortingMode sortingMode);
}