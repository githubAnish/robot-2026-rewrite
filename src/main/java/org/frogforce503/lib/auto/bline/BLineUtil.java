package org.frogforce503.lib.auto.bline;

import java.util.ArrayList;
import java.util.List;

import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

public class BLineUtil {
    private BLineUtil() {}

    public static FollowPath.Builder configureAutoBuilder(Drive drive) {
        return
            new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getRobotVelocity,
                drive::runVelocity,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0)
            )
            .withDefaultShouldFlip()
            .withPoseReset(drive::setPose);
    }

    public static Pose2d[] getPoses(Path... paths) {
        List<Pose2d> poses = new ArrayList<>();

        for (Path path : paths) {
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
}