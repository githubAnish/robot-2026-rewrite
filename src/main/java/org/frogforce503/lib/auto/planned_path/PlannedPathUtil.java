package org.frogforce503.lib.auto.planned_path;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;

public final class PlannedPathUtil {
    private PlannedPathUtil() {}

    public static Pose2d[] getPoses(PlannedPath... plannedPaths) {
        return
            Arrays
                .stream(plannedPaths)
                .flatMap(path -> path.getDriveTrajectory().getStates().stream())
                .map(state -> state.poseMeters)
                .toArray(Pose2d[]::new);
    }
}