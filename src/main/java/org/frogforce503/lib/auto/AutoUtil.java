package org.frogforce503.lib.auto;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.robot.commands.drive.DriveToPose;
import org.frogforce503.robot.subsystems.drive.Drive;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;

public final class AutoUtil {
    private AutoUtil() {}

    /**
     * Returns the current robot pose with a fallback for simulation.
     * Only required in autos that start with a {@link PlannedPath} trajectory or {@link DriveToPose} command.
     */
    public static Pose2d setupInitialPose(Drive drive, Pose2d simulationFallback) {
        return RobotBase.isReal() ? drive.getPose() : simulationFallback;
    }

    public static List<Pose2d> getPoses(PlannedPath... plannedPaths) {
        return
            Arrays
                .stream(plannedPaths)
                .flatMap(path -> path.getDriveTrajectory().getStates().stream())
                .map(state -> state.poseMeters)
                .toList();
    }

    public static List<Pose2d> getPoses(AutoTrajectory... choreoTrajectories) {
        return
            Arrays
                .stream(choreoTrajectories)
                .flatMap(traj -> Arrays.stream(traj.getRawTrajectory().getPoses()))
                .collect(Collectors.toList());
    }
}
