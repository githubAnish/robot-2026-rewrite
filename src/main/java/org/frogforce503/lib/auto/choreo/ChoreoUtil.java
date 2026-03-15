package org.frogforce503.lib.auto.choreo;

import java.util.Arrays;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ChoreoUtil {
    private ChoreoUtil() {}

    public static AutoFactory createAutoFactory(Drive drive) {
        return
            new AutoFactory(
                drive::getPose,
                drive::setPose,
                sample -> followTrajectory(drive, (SwerveSample) sample),
                Constants.useAllianceFlipping,
                drive);
    }

    private static void followTrajectory(Drive drive, SwerveSample sample) {
        // Generate the next robot-relative speeds for the robot
        ChassisSpeeds speeds =
            DriveConstants.pathFollower.calculate(
                drive.getPose(),
                sample.getPose(),
                sample.vx,
                sample.vy,
                sample.omega);

        // Apply the generated speeds (with module forces)
        drive.runVelocity(
            speeds,
            sample.moduleForcesX(),
            sample.moduleForcesY());
    }

    public static Pose2d[] getPoses(AutoTrajectory... choreoTrajectories) {
        return
            Arrays
                .stream(choreoTrajectories)
                .flatMap(traj -> Arrays.stream(traj.getRawTrajectory().getPoses()))
                .toArray(Pose2d[]::new);
    }
}
