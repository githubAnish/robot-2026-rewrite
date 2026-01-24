package org.frogforce503.robot.subsystems.vision.apriltagdetection;

import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * An implementation of the AprilTagIO interface.
 * 
 * It extends AprilTagIOPhotonVision to simulate AprilTag detection with a PhotonVision camera using PhotonSim.
 */
public class AprilTagIOPhotonSim extends AprilTagIOPhotonVision {
    private PhotonCameraSim cameraSim;
    private VisionSystemSim aprilTagDetectionSimulator;

    /**
     * @param cameraName The enum representing the name of the camera configured in PhotonVision
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin
     * @param visionSimulator The VisionSimulator that contains the VisionSystemSim instance that the camera will be added – the simulation world for the camera.
     * @param cameraProperties The SimCameraProperties to configure the camera simulation
     */
    public AprilTagIOPhotonSim(
        CameraName cameraName, 
        Transform3d robotToCameraOffset, 
        VisionSimulator visionSimulator, 
        SimCameraProperties cameraProperties
    ) {
        super(cameraName, robotToCameraOffset, visionSimulator.getAprilTagFieldLayout());

        cameraSim = new PhotonCameraSim(super.getCamera(), cameraProperties,visionSimulator.getAprilTagFieldLayout());   
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        this.aprilTagDetectionSimulator = visionSimulator.getAprilTagDetectionSimulator();
        aprilTagDetectionSimulator.addCamera(cameraSim, robotToCameraOffset);
    }

    /**
     * @param cameraName The enum representing the name of the camera configured in PhotonVision
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin
     * @param visionSimulator The VisionSimulator that contains the VisionSystemSim instance that the camera will be added – the simulation world for the camera.
     */
    public AprilTagIOPhotonSim(
        CameraName cameraName, 
        Transform3d robotToCameraOffset, 
        VisionSimulator visionSimulator
    ) {
        this(
            cameraName, 
            robotToCameraOffset, 
            visionSimulator, 
            new SimCameraProperties()
                .setCalibration(1280, 800, Rotation2d.fromDegrees(78.2))
                .setCalibError(0.25, 0.08)
                .setFPS(30)
                .setAvgLatencyMs(15)
                .setLatencyStdDevMs(5)
        );
    }

    @Override
    public void setRobotToCameraOffset(Transform3d robotToCameraOffset) {
        super.setRobotToCameraOffset(robotToCameraOffset);
        aprilTagDetectionSimulator.adjustCamera(cameraSim, robotToCameraOffset);
    }
}
