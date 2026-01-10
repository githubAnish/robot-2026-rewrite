package org.frogforce503.robot.subsystems.vision.apriltag_detection;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.frogforce503.lib.vision.apriltag_detection.*;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import lombok.Getter;

/**
 * An implementation of the AprilTagIO interface.
 * 
 * Represents a camera running PhotonVision for AprilTag detection
 */
public class AprilTagIOPhotonVision implements AprilTagIO {
    private CameraName cameraName;

    @Getter private PhotonCamera camera; // Represents the camera used for AprilTag detection.
    private Transform3d robotToCameraOffset;

    private PhotonPoseEstimator poseEstimator; // The PhotonPoseEstimator is used to estimate the robot's pose based on the camera's latest result. It can use different strategies.

    private PhotonPipelineResult latestResult; // The latest result from the PhotonCamera, which contains information about detected AprilTags.
    private List<PhotonTrackedTarget> allTrackedAprilTags; // The list of tracked april tags from the latest result, including ignored ones.
    private EstimatedRobotPose lastEstimatedRobotPose; // The last estimated robot pose from the pose estimator.

    // Corresponds to pose strategies used by the PhotonPoseEstimator.
    private PoseObservationType primaryPoseObservationType;
    private PoseObservationType secondaryPoseObservationType;

    Set<Integer> ignoredAprilTagIDs = new HashSet<>(); //Set of AprilTag IDs that should be ignored for pose estimation.
    
    //Constructors
    /**
     * @param cameraName The enum representing name of the camera configured in PhotonVision
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin
     * @param aprilTagFieldLayout The AprilTagFieldLayout to use for pose estimation
    */
    public AprilTagIOPhotonVision(CameraName cameraName, Transform3d robotToCameraOffset, AprilTagFieldLayout aprilTagFieldLayout) {
        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName.name());
        this.robotToCameraOffset = robotToCameraOffset;

        poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCameraOffset
        );

        setPoseObservationType(PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR);
    }

    /**
     * @param cameraName The enum representing name of the camera configured in PhotonVision
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin
    */
    public AprilTagIOPhotonVision(CameraName cameraName, Transform3d robotToCameraOffset) {
        this(cameraName, robotToCameraOffset, Constants.fieldVenue.getAprilTagFieldLayout());
    }

    //VisionIO
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
        poseEstimator.setRobotToCameraTransform(robotToCameraOffset);
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
            return -1; //Return an invalid index if the camera is not connected
        }
    }

    // AprilTagIO
    @Override
    public void updateInputs(AprilTagInputs inputs) {
        inputs.connected = camera.isConnected();

        // Persist old results for up to 100 ms
        if (latestResult != null && Timer.getFPGATimestamp() - latestResult.getTimestampSeconds() > 0.1) { 
            // Default values for inputs when reset
            inputs.persistingOldResults = false;
            latestResult = null;
            allTrackedAprilTags = null;
            inputs.hasTargets = false;
            inputs.trackedAprilTags = new TrackedAprilTag[0];
            lastEstimatedRobotPose = null;
        } else {
            inputs.persistingOldResults = true;
        }

        if (inputs.connected) {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            if (!results.isEmpty()) {
                int i = results.size() - 1;
                PhotonPipelineResult result = results.get(i); // Get the most recent result

                while (!result.hasTargets() && i > 0) {
                    i--;
                    result = results.get(i); // Get the most recent result with targets
                }

                // Don't persist an old result if it doesn't have any targets or if the new result has targets
                if (latestResult == null || !latestResult.hasTargets() || (latestResult.hasTargets() && result.hasTargets())) {
                    latestResult = result;
                    inputs.persistingOldResults = false;
                }

                // Save original list of targets
                allTrackedAprilTags = latestResult.getTargets();

                inputs.hasTargets = latestResult.hasTargets();
                inputs.trackedAprilTags = latestResult.targets.stream()
                    .map(tag -> new TrackedAprilTag(
                            tag.getFiducialId(),
                            tag.getPitch(),
                            tag.getYaw(),
                            tag.getArea(),
                            tag.getBestCameraToTarget().getTranslation().getNorm(),
                            tag.getPoseAmbiguity()
                            )
                        )
                    .toArray(TrackedAprilTag[]::new);
            }
        }
    }

    @Override
    public PoseObservation estimateRobotPose() {
        PoseObservation poseObservation = new PoseObservation();

        if (latestResult != null && allTrackedAprilTags != null) { // Only use pose estimator if there are tracked AprilTags
            latestResult.targets = allTrackedAprilTags.stream()
                .filter(tag -> !ignoredAprilTagIDs.contains(tag.getFiducialId())) // Filter out ignored tags
                .toList();

            Optional<EstimatedRobotPose> optionalRobotPose = poseEstimator.update(latestResult);

            if (optionalRobotPose.isPresent()) {
                lastEstimatedRobotPose = optionalRobotPose.get(); 
            }

            if (lastEstimatedRobotPose != null) {
                poseObservation = new PoseObservation(
                    lastEstimatedRobotPose.timestampSeconds,
                    lastEstimatedRobotPose.estimatedPose,
                    lastEstimatedRobotPose.targetsUsed.size() > 1 ? primaryPoseObservationType : secondaryPoseObservationType, // Use primary if multiple tags are used, otherwise use secondary
                    lastEstimatedRobotPose.targetsUsed.stream()
                        .map(tag -> new TrackedAprilTag(
                                tag.getFiducialId(),
                                tag.getPitch(),
                                tag.getYaw(),
                                tag.getArea(),
                                tag.getBestCameraToTarget().getTranslation().getNorm(),
                                tag.getPoseAmbiguity()
                                )
                            )
                        .toArray(TrackedAprilTag[]::new)
                );
            }

            latestResult.targets = allTrackedAprilTags.stream().toList(); // Restore the original list of targets
        }

        return poseObservation;
    }

    @Override
    public void setPoseObservationType(PoseObservationType poseObservationType) {
        switch (poseObservationType) {
            case MEGATAG1:
                System.out.println("Not valid for PhotonVision");
                break;
            case MEGATAG2:
                System.out.println("Not valid for PhotonVision");
                break;
            case MULTI_TAG_PNP_ON_COPROCESSOR: //PhotonVision PoseObservationType that requires a multi-tag fallback
                poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE); //default fallback strategy

                primaryPoseObservationType = poseObservationType;
                secondaryPoseObservationType = PoseObservationType.CLOSEST_TO_REFERENCE_POSE; //Set the secondary pose observation type to the fallback strategy
                break;
            default:
                poseEstimator.setPrimaryStrategy(PoseStrategy.valueOf(poseObservationType.name()));
                poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.valueOf(poseObservationType.name()));

                primaryPoseObservationType = poseObservationType;
                secondaryPoseObservationType = poseObservationType;
                break;
        }
    }

    @Override
    public void setSecondaryPoseObservationType(PoseObservationType poseObservationType) {
        secondaryPoseObservationType = poseObservationType;

        switch (poseObservationType) {
            case MEGATAG1:
                System.out.println("Not valid for PhotonVision");
                break;
            case MEGATAG2:
                System.out.println("Not valid for PhotonVision");
                break;
            case MULTI_TAG_PNP_ON_COPROCESSOR:
                System.out.println("Not valid for PhotonVision");
                break;
            default: // Secondary PoseObservationType is a multi-tag fallback strategy for PhotonVision.
                poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.valueOf(poseObservationType.name()));
                break;
        }
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        poseEstimator.setReferencePose(pose);
        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), pose.getRotation());
    }

    @Override
    public void setIgnoredAprilTags(Set<Integer> ids) {
        ignoredAprilTagIDs = ids;
    }
}