package org.frogforce503.lib.vision;

import org.frogforce503.lib.vision.apriltag_detection.TrackedAprilTag;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Utility class for vision with useful static methods.
 */
public class VisionUtils {
    /**
     * Returns the lowest ambiguity of all tracked AprilTags.
     * 
     * @param trackedAprilTags The array of tracked AprilTags
     * @return The lowest ambiguity value
     */
    public static double getLowestAmbiguity(TrackedAprilTag[] trackedAprilTags) {
        double lowestAmbiguity = Double.MAX_VALUE;
        for (TrackedAprilTag trackedAprilTag : trackedAprilTags) {
            if (trackedAprilTag.ambiguity() < lowestAmbiguity) {
                lowestAmbiguity = trackedAprilTag.ambiguity();
            }
        }
        return lowestAmbiguity;
    }

    /**
     * Returns the average ambiguity of all tracked AprilTags.
     * @param trackedAprilTags The array of tracked AprilTags
     * @return The average ambiguity value, or Double.MAX_VALUE if no tags are tracked
     */
    public static double getAverageAmbiguity(TrackedAprilTag[] trackedAprilTags) {
        double lowestAmbiguity = 0;

        for (TrackedAprilTag trackedAprilTag : trackedAprilTags) {
            lowestAmbiguity += trackedAprilTag.ambiguity();
        }

        return (trackedAprilTags.length == 0) ? Double.MAX_VALUE : lowestAmbiguity/trackedAprilTags.length;
    }

    /**
     * Returns the lowest distance to camera of all tracked AprilTags.
     * 
     * @param trackedAprilTags The array of tracked AprilTags
     * @return The lowest distance to camera value
     */
    public static double getLowestDistanceToCamera(TrackedAprilTag[] trackedAprilTags) {
        double lowestDistanceToCamera = Double.MAX_VALUE;
        for (TrackedAprilTag trackedAprilTag : trackedAprilTags) {
            if (trackedAprilTag.distance() < lowestDistanceToCamera) {
                lowestDistanceToCamera = trackedAprilTag.distance();
            }
        }
        return lowestDistanceToCamera;
    }

    /**
     * Returns the average distance to camera of all tracked AprilTags.
     * 
     * @param trackedAprilTags The array of tracked AprilTags
     * @return The average distance to camera value, or Double.MAX_VALUE if no tags are tracked
     */
    public static double getAverageDistanceToCamera(TrackedAprilTag[] trackedAprilTags) {
        double lowestDistanceToCamera = 0;

        for (TrackedAprilTag trackedAprilTag : trackedAprilTags) {
            lowestDistanceToCamera += trackedAprilTag.distance();
        }
        
        return (trackedAprilTags.length == 0) ? Double.MAX_VALUE : lowestDistanceToCamera/trackedAprilTags.length;
    }
    
    /**
     * @param robotToCameraOffset The offset from the robot to the camera in meters and radians
     * @param objectHeight The height of the object in meters
     * @param objectPitch The pitch of the object in radians
     * @param objectYaw The yaw of the object in radians
     * 
     * @return The 2d translation from the center of the robot (x is forward, y is left) to the object in meters
    */
    public static Translation2d getRobotToObject(Transform3d robotToCameraOffset, double objectHeight, double objectPitch, double objectYaw) {
        double cameraToObjectDistance = PhotonUtils.calculateDistanceToTargetMeters(robotToCameraOffset.getZ(), objectHeight, robotToCameraOffset.getRotation().getY(), objectYaw);
        Translation2d cameraToObject = PhotonUtils.estimateCameraToTargetTranslation(cameraToObjectDistance, new Rotation2d(-objectYaw));
        Translation2d robotToObject =  cameraToObject.plus(robotToCameraOffset.getTranslation().toTranslation2d());

        return robotToObject;
    }

    /**
     * @param robotPose The pose of the robot in the field in meters and radians
     * @param robotToCameraOffset The offset from the robot to the camera in meters and radians
     * @param objectHeight The height of the object in meters
     * @param objectPitch The pitch of the object in radians
     * @param objectYaw The yaw of the object in radians
     * 
     * @return the 2d translation from the origin of the field to the object in meters
    */
    public static Translation2d getFieldToObject(Pose2d robotPose, Transform3d robotToCameraOffset, double objectHeight, double objectPitch, double objectYaw) {
        Translation2d robotToObject =  getRobotToObject(robotToCameraOffset, objectHeight, objectPitch, objectYaw);
        Translation2d fieldToObject = robotToObject.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());

        return fieldToObject;
    }
}
