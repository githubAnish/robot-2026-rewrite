package org.frogforce503.robot.subsystems.vision.io.apriltagdetection;

import java.util.List;
import java.util.Optional;

import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraConfig;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AprilTagIOPhotonVision implements AprilTagIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public AprilTagIOPhotonVision(CameraConfig config) {
        camera = new PhotonCamera(config.name());
        poseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, config.robotToCamera());
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

        for (var result : camera.getAllUnreadResults()) {
            estimatedRobotPose = poseEstimator.estimateCoprocMultiTagPose(result);

            if (estimatedRobotPose.isEmpty()) {
                estimatedRobotPose = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            estimatedRobotPose.ifPresent(
                est -> {
                    inputs.timestamp = est.timestampSeconds;
                    inputs.estimatedRobotPose = est.estimatedPose;
                });

            inputs.estimatedStdDevs = getEstimationStdDevs(estimatedRobotPose, result.getTargets());
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            return VisionConstants.kSingleTagStdDevs;
        }
        
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double averageDist = 0;

        for (var target : targets) {
            var tagPose =
                FieldConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId());

            if (tagPose.isEmpty()) {
                continue;
            }

            numTags++;

            averageDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            return VisionConstants.kSingleTagStdDevs;
        }

        averageDist /= numTags;

        if (numTags > 1) {
            estStdDevs = VisionConstants.kMultiTagStdDevs;
        }

        if (numTags == 1 && averageDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (averageDist * averageDist / 30));
        }
        
        return estStdDevs;
    }
}