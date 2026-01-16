package org.frogforce503.lib.rebuilt;

import java.util.Arrays;

import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public final class ProximityUtil {
    private ProximityUtil() {}

    // General-purpose proximity methods
    public static double getDistanceBetweenPoses(Pose2d first, Pose2d second) {
        return first.minus(second).getTranslation().getNorm();
    }

    public static double getDistanceFromPose(Drive drive, Pose2d target) {
        return getDistanceBetweenPoses(drive.getPose(), target);
    }

    public static Pose2d getClosestPose(Drive drive, Pose2d... options) {
        if (options.length == 0) {
            return null;
        }

        return drive.getPose().nearest(Arrays.asList(options));
    }

    // Rebuilt-specific proximity methods
}
