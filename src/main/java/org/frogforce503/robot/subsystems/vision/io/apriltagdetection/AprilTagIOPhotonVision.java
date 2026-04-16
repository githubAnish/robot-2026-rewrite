package org.frogforce503.robot.subsystems.vision.io.apriltagdetection;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.lib.vision.apriltagdetection.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import lombok.Getter;

public class AprilTagIOPhotonVision implements AprilTagIO {
    @Getter private final PhotonCamera camera;
    @Getter private final CameraName cameraName;
    @Getter private Transform3d robotToCameraOffset;

    private final PhotonPoseEstimator poseEstimator;
    
    private PhotonPipelineResult latestResult;
    private List<PhotonTrackedTarget> allTrackedAprilTags;
    private EstimatedRobotPose lastEstimatedRobotPose;

    private PoseObservationType primaryPoseObservationType;
    private PoseObservationType secondaryPoseObservationType;

    private Set<Integer> ignoredAprilTagIDs = new HashSet<>();
    
    public AprilTagIOPhotonVision(CameraName cameraName) {
        this.camera = new PhotonCamera(cameraName.name());
        this.cameraName = cameraName;
        this.robotToCameraOffset = VisionConstants.robotToFixedCameraOffsets.get(cameraName);

        poseEstimator =
            new PhotonPoseEstimator(
                FieldConstants.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraOffset);

        setPoseObservationType(PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR);
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
            return -1;
        }
    }

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
                PhotonPipelineResult result = results.get(results.size() - 1); // Get most recent result

                // Remove tags that are absolutely horrible to detect to avoid unnecessary logging while processing inputs
                for (int i = result.targets.size() - 1; i >= 0; i--) {
                    PhotonTrackedTarget aprilTag = result.getTargets().get(i);

                    boolean tooAmbiguous = aprilTag.getPoseAmbiguity() > VisionConstants.ABSOLUTE_MAX_AMBIGUITY;
                    boolean tooFar = aprilTag.getBestCameraToTarget().getTranslation().getNorm() > VisionConstants.ABSOLUTE_MAX_DISTANCE_TO_TAG;

                    if (tooAmbiguous || tooFar) {
                        result.targets.remove(i);
                    }
                }

                // Don't persist old result if it doesn't have any targets or if the new result has targets
                if (latestResult == null || !latestResult.hasTargets() || (latestResult.hasTargets() && result.hasTargets())) {
                    latestResult = result;
                    inputs.persistingOldResults = false;
                }

                // Save original list of targets
                allTrackedAprilTags = latestResult.getTargets();

                inputs.hasTargets = latestResult.hasTargets();
                inputs.trackedAprilTags =
                    latestResult.targets
                        .stream()
                        .map(
                            tag ->
                                new TrackedAprilTag(
                                    tag.getFiducialId(),
                                    tag.getPitch(),
                                    tag.getYaw(),
                                    tag.getArea(),
                                    tag.getBestCameraToTarget().getTranslation().getNorm(),
                                    tag.getPoseAmbiguity())
                        )
                        .toArray(TrackedAprilTag[]::new);
            }
        }
    }

    @Override
    public PoseObservation estimateRobotPose() {
        PoseObservation poseObservation = PoseObservation.kZero;

        if (latestResult != null && allTrackedAprilTags != null) { // Only use pose estimator if there are tracked AprilTags
            latestResult.targets =
                allTrackedAprilTags
                    .stream()
                    .filter(tag -> !ignoredAprilTagIDs.contains(tag.getFiducialId())) // Filter out ignored tags
                    .toList();

            Optional<EstimatedRobotPose> optionalRobotPose = poseEstimator.update(latestResult);

            if (optionalRobotPose.isPresent()) {
                lastEstimatedRobotPose = optionalRobotPose.get(); 
            }

            if (lastEstimatedRobotPose != null) {
                poseObservation =
                    new PoseObservation(
                        lastEstimatedRobotPose,
                        lastEstimatedRobotPose.targetsUsed.size() > 1
                            ? primaryPoseObservationType
                            : secondaryPoseObservationType);
            }

            latestResult.targets = allTrackedAprilTags.stream().toList(); // Restore original list of targets
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

            case MULTI_TAG_PNP_ON_COPROCESSOR: // PhotonVision PoseObservationType that requires a multi-tag fallback
                poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE); // default fallback strategy

                primaryPoseObservationType = poseObservationType;
                secondaryPoseObservationType = PoseObservationType.CLOSEST_TO_REFERENCE_POSE; // Set the secondary pose observation type to the fallback strategy
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