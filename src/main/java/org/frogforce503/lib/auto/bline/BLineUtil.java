package org.frogforce503.lib.auto.bline;

import java.util.ArrayList;
import java.util.List;

import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

public final class BLineUtil {
    private BLineUtil() {}

    public static FollowPath.Builder configureAutoBuilder(Drive drive) {
        return
            new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getRobotVelocity,
                drive::runVelocity,
                DriveConstants.blineLinearPID.toPIDController(),
                DriveConstants.blineThetaPID.toPIDController(),
                DriveConstants.blineCtePID.toPIDController()
            )
            .withDefaultShouldFlip();
    }

    public static Pose2d[] getPoses(Path... paths) {
        List<Pose2d> poses = new ArrayList<>();

        for (Path path : paths) {
            // Flip poses if on red alliance
            if (FieldConstants.isRed()) {
                path.flip();
            }

            // Add pose based on element type
            for (PathElement element : path.getPathElements()) {
                if (element instanceof Waypoint w) {
                    poses.add(new Pose2d(w.translationTarget().translation(), w.rotationTarget().rotation()));
                    
                } else if (element instanceof TranslationTarget t) {
                    poses.add(new Pose2d(t.translation(), Rotation2d.kZero));
                    
                } else if (element instanceof RotationTarget r) {
                    poses.add(new Pose2d(Translation2d.kZero, r.rotation()));
                }
            }
        }

        return poses.toArray(Pose2d[]::new);
    }

    public static Pose2d[] simulateTrail(
        List<Translation2d> waypoints,
        double handoffRadius,
        double maxVel,
        double maxAcc,
        double dt
    ) {
        List<Pose2d> trail = new ArrayList<>();

        if (waypoints.size() < 2) return new Pose2d[0];

        Translation2d pos = waypoints.get(0);
        trail.add(new Pose2d(pos, new Rotation2d()));

        double vel = 0.0;
        int currentTarget = 1;

        while (currentTarget < waypoints.size()) {
            Translation2d target = waypoints.get(currentTarget);
            boolean isLast = (currentTarget == waypoints.size() - 1);
            double distToTarget = pos.getDistance(target);

            // Handoff — advance target early if within radius and not last
            if (!isLast && distToTarget < handoffRadius) {
                currentTarget++;
                continue;
            }

            // Remaining distance through all upcoming waypoints
            double remaining = distToTarget;
            for (int i = currentTarget; i < waypoints.size() - 1; i++) {
                remaining += waypoints.get(i).getDistance(waypoints.get(i + 1));
            }

            // Speed from kinematic formula, rate limited
            double desiredVel = Math.min(maxVel, Math.sqrt(2.0 * maxAcc * remaining));
            double maxDelta = maxAcc * dt;
            vel = Math.clamp(desiredVel, vel - maxDelta, vel + maxDelta);

            // Step toward target
            Translation2d dir = target.minus(pos);
            double dist = dir.getNorm();

            if (dist < 1e-6) {
                currentTarget++;
                continue;
            }

            Translation2d unitDir = dir.div(dist);
            double step = vel * dt;

            if (step >= dist) {
                pos = target;
                if (isLast) {
                    trail.add(new Pose2d(pos, new Rotation2d()));
                    break;
                }
                currentTarget++;
            } else {
                pos = pos.plus(unitDir.times(step));
            }

            trail.add(new Pose2d(pos, new Rotation2d()));
        }

        return trail.toArray(new Pose2d[0]);
    }
}