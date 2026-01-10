package org.frogforce503.robot.subsystems.vision.object_detection;

import java.util.Comparator;
import java.util.List;

import org.frogforce503.lib.vision.object_detection.TrackedObject;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * An implementation of the ObjectDetectionIO interface.
 * 
 * It extends ObjectDetectionIOPhotonVision to simulate object detection with a PhotonVision camera using PhotonSim.
 */
public class ObjectDetectionIOPhotonSim extends ObjectDetectionIOPhotonVision {
    private PhotonCameraSim cameraSim;
    private VisionSystemSim objectDetectionSimulator;

    /**
     * @param cameraName The enum representing the name of the camera configured in PhotonVision.
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin.
     * @param visionSimulator The VisionSimulator that contains the VisionSystemSim instance that the camera will be added – the simulation world for the camera.
     * @param cameraProperties The SimCameraProperties to configure the camera simulation.
     */
    public ObjectDetectionIOPhotonSim(
        CameraName cameraName, 
        Transform3d robotToCameraOffset, 
        VisionSimulator visionSimulator, 
        SimCameraProperties cameraProperties
    ) {
        super(cameraName, robotToCameraOffset);

        PhotonCameraSim cameraSim = new PhotonCameraSim(super.getCamera(), cameraProperties);   
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        this.objectDetectionSimulator = visionSimulator.getObjectDetectionSimulator();
        objectDetectionSimulator.addCamera(cameraSim, robotToCameraOffset);
    }
    
    /**
     *
     * @param cameraName The enum representing the name of the camera configured in PhotonVision.
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin.
     * @param visionSimulator The VisionSimulator that contains the VisionSystemSim instance that the camera will be added – the simulation world for the camera.
     */
    public ObjectDetectionIOPhotonSim(CameraName cameraName, Transform3d robotToCameraOffset, VisionSimulator visionSimulator) {
        this(cameraName, 
            robotToCameraOffset, 
            visionSimulator, 
            new SimCameraProperties()
                .setCalibration(640, 480, Rotation2d.fromDegrees(127.83))
                .setCalibError(0.25, 0.08)
                .setFPS(20)
                .setAvgLatencyMs(35)
                .setLatencyStdDevMs(5)
        );
    }

    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        super.updateInputs(inputs);

        /*
        PhotonSim automatically sets the class id for simulated targets to -1, they only use the fiducial id.
        In the VisionSimulator class, we set the fiducial id to be the class id for simulated targets.
        When updating the list of TrackedObjects in the inputs, we map the fiducial id to the class id.
        */
        if (super.getLatestResult() != null && inputs.trackedObjects.length > 0) {
            List<PhotonTrackedTarget> objects = super.getLatestResult().getTargets();

            inputs.trackedObjects = objects.stream()
                .map(object -> new TrackedObject( // Maps the PhotonTrackedTarget to a TrackedObject.
                    object.getFiducialId(),
                    object.getPitch(),
                    object.getYaw(),
                    object.getArea(),
                    1.0 // Simulated targets do not have a confidence value, so we set it to 100%
                ))
                .sorted( // Sorts based on the type of object (class ID) and then by the sorting mode.
                    Comparator.comparingInt(TrackedObject::classId)
                    .thenComparing(super.getSortingMode().getComparator())
                )
                .toArray(TrackedObject[]::new);
        }
    }
    
    @Override
    public void setRobotToCameraOffset(Transform3d robotToCameraOffset) {
        super.setRobotToCameraOffset(robotToCameraOffset);
        objectDetectionSimulator.adjustCamera(cameraSim, robotToCameraOffset);
    }
}
