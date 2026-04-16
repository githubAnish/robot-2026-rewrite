package org.frogforce503.robot.subsystems.vision.io.apriltagdetection;

import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.robot.viz.VisionSimulator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagIOPhotonSim extends AprilTagIOPhotonVision {
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim aprilTagDetectionSimulator;

    public AprilTagIOPhotonSim(CameraName cameraName, VisionSimulator visionSimulator, SimCameraProperties cameraProperties) {
        super(cameraName);

        cameraSim = new PhotonCameraSim(super.getCamera(), cameraProperties, FieldConstants.aprilTagFieldLayout);   
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        aprilTagDetectionSimulator = visionSimulator.getAprilTagDetectionSimulator();
        aprilTagDetectionSimulator.addCamera(cameraSim, super.getRobotToCameraOffset());
    }

    public AprilTagIOPhotonSim(CameraName cameraName, VisionSimulator visionSimulator) {
        this(
            cameraName,  
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