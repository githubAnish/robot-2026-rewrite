package org.frogforce503.lib.auto.planned_path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.frogforce503.lib.auto.planned_path.components.Waypoint;
import org.frogforce503.lib.auto.planned_path.generator.CustomTrajectoryGenerator;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/** Wrapper class for the {@link CustomTrajectoryGenerator} to create a {@link PlannedPath} easier. */
public class PlannedPathFactory {
    private static final CustomTrajectoryGenerator trajectoryGenerator = new CustomTrajectoryGenerator();

    private PlannedPathFactory() {}

    public static TrajectoryConfig config(double vMax, double aMax, double vInitial, double vFinal) {
        return
            new TrajectoryConfig(vMax, aMax)
                .setKinematics(DriveConstants.kinematics)
                .setStartVelocity(vInitial)
                .setEndVelocity(vFinal);
    }

    public static PlannedPath generate(TrajectoryConfig config, List<Waypoint> waypoints) {
        try {
            trajectoryGenerator.generate(config, waypoints);
        } catch (TrajectoryGenerationException e) {
            System.out.print("Trajectory generation failed" + ErrorUtil.attachJavaClassName(PlannedPathFactory.class));
            e.printStackTrace();
            return null;
        }
        
        return trajectoryGenerator.getPlannedPath(waypoints);
    }

    public static PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, List<Waypoint> waypoints) {
        return
            generate(
                config(vMax, aMax, vInitial, vFinal),
                waypoints);
    }

    public static PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, Waypoint... waypoints) {
        return generate(vMax, aMax, vInitial, vFinal, Arrays.asList(waypoints));
    }

    /** Re-generate an existing {@link PlannedPath} with a new trajectory configuration. */
    public static PlannedPath regenerate(PlannedPath existing, double vMax, double aMax, double vi, double vf) {
        return generate(vMax, aMax, vi, vf, existing.getWaypoints());
    }

    /** Returns a reversed version of {@code original}, with all waypoints in reverse order. */
    public static PlannedPath reversedOf(PlannedPath original, double vMax, double aMax, double vi, double vf) {
        List<Waypoint> waypoints = original.getWaypoints();
        List<Waypoint> reversedWaypoints = new ArrayList<>();

        for (int i = waypoints.size() - 1; i >= 0; i--) {
            Waypoint waypoint = waypoints.get(i);
            
            if (waypoint.getDriveRotation().isPresent()) {
                waypoint =
                    new Waypoint(
                        waypoint.getTranslation(),
                        waypoint
                            .getDriveRotation()
                            .get()
                            .plus(Rotation2d.kPi),
                        waypoint
                            .getHolonomicRotation()
                            .orElse(null));
            }

            reversedWaypoints.add(waypoint);
        }

        return generate(vMax, aMax, vi, vf, reversedWaypoints);
    }
}
