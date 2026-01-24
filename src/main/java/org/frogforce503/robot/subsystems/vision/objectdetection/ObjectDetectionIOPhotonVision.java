package org.frogforce503.robot.subsystems.vision.objectdetection;

import java.util.Comparator;
import java.util.List;

import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.lib.vision.objectdetection.ObjectSortingMode;
import org.frogforce503.lib.vision.objectdetection.TrackedObject;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;

/**
 * An implementation of the ObjectDetectionIO interface.
 * 
 * Represents a camera running PhotonVision for object detection.
 */
public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
    private CameraName cameraName;

    @Getter private PhotonCamera camera; //Represents the camera used for object detection.
    private Transform3d robotToCameraOffset;

    @Getter private PhotonPipelineResult latestResult; //Contains all the data from the latest result outputted by the camera.

    @Getter @Setter private ObjectSortingMode sortingMode = ObjectSortingMode.Centermost;
    
    /**
     * @param cameraName The enum representing name of the camera configured in PhotonVision.
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin.
    */
    public ObjectDetectionIOPhotonVision(CameraName cameraName, Transform3d robotToCameraOffset) {
        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName.name());
        this.robotToCameraOffset = robotToCameraOffset;
    }

    //Vision IO
    @Override
    public CameraName getCameraName() {
        return cameraName;
    }

    @Override
    public Transform3d getRobotToCameraOffset() {
        return robotToCameraOffset;
    }

    @Override
    public void setRobotToCameraOffset(Transform3d robotToCameraOffset) {
        this.robotToCameraOffset = robotToCameraOffset;
    }

    @Override 
    public void setPipeline(int pipelineIndex) {
        if (camera.isConnected() && pipelineIndex >= 0) {
            camera.setPipelineIndex(pipelineIndex);
        }
    }; 

    @Override
    public int getPipeline() {
        if (camera.isConnected()) {
            return camera.getPipelineIndex();
        } else {
            return -1; // Returns an invalid index if the camera is not connected.
        }
    }

    //ObjectDetectionIO
    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        inputs.connected = camera.isConnected();

        if (latestResult != null && Timer.getFPGATimestamp() - latestResult.getTimestampSeconds() > 0.1) { //Persist results for 1 second/20 frames per second
            //Default values for inputs
            inputs.persistingOldResults = false;
            latestResult = null;
            inputs.hasTargets = false;
            inputs.trackedObjects = new TrackedObject[0];
        } else {
            inputs.persistingOldResults = true;
        }
        
        if (inputs.connected) {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            if (!results.isEmpty()) {
                int i = results.size() - 1;
                PhotonPipelineResult result = results.get(i); // Get the most recent result
                inputs.persistingOldResults = false;

                while (!result.hasTargets() && i > 0) {
                    i--;
                    result = results.get(i); // Get the most recent result with targets
                }

                if (latestResult == null || !latestResult.hasTargets() || (latestResult.hasTargets() && result.hasTargets())) {
                    latestResult = result;
                    inputs.persistingOldResults = false;
                }
                
                inputs.hasTargets = latestResult.hasTargets();
            }

            if (inputs.hasTargets) {
                List<PhotonTrackedTarget> objects = latestResult.getTargets();
    
                inputs.trackedObjects = inputs.trackedObjects = objects.stream()
                .map(object -> new TrackedObject( // Maps the PhotonTrackedTarget to a TrackedObject.
                    object.getDetectedObjectClassID(),
                    object.getPitch(),
                    object.getYaw(),
                    object.getArea(),
                    (double) object.getDetectedObjectConfidence()
                ))
                .sorted( // Sorts based on the type of object (class ID) and then by the sorting mode.
                    Comparator.comparingInt(TrackedObject::classId)
                    .thenComparing(sortingMode.getComparator())
                )
                .toArray(TrackedObject[]::new);
    
                if (inputs.persistingOldResults) inputs.persistingOldResults = false;
            } 
        } 
    }
}