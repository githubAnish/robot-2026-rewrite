package org.frogforce503.lib.auto.choreo;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ChoreoUtil {
    private ChoreoUtil() {}

    public static AutoFactory createAutoFactory(Drive drive) {
        return
            new AutoFactory(
                drive::getPose,
                drive::setPose,
                trajectorySample -> {
                    SwerveSample sample = (SwerveSample) trajectorySample;

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
                },
                Constants.useAllianceFlipping,
                drive);
    }
}
