package org.frogforce503.robot.commands.drive;

import java.util.function.Supplier;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.lib.auto.planned_path.PlannedPath.HolonomicState;
import org.frogforce503.lib.swerve.SwervePathController;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Setter;

public class DrivePlannedPath extends Command {
    // Requirements
    private final Drive drive;

    // Control
    private final SwervePathController controller = DriveConstants.pathFollower;
    private final Timer timer;
    private final PlannedPath trajectory;

    // State
    private final boolean willStopAtEnd;

    // Overrides
    @Setter private Supplier<Rotation2d> headingOverride;

    public DrivePlannedPath(Drive drive, PlannedPath trajectory) {
        this.drive = drive;

        this.timer = new Timer();
        this.trajectory = trajectory;

        this.willStopAtEnd =
            trajectory
                .getDriveTrajectory()
                .sample(trajectory.getTotalTimeSeconds())
                .velocityMetersPerSecond == 0.00; // originally 0.1

        addRequirements(drive);
    }

    @Override
    public void initialize() { 
        timer.reset();
        controller.reset();
        timer.start();

        var poses =
            trajectory
                .getDriveTrajectory()
                .getStates()
                .stream()
                .map(state -> state.poseMeters)
                .toArray(Pose2d[]::new);

        drive.getViz()
            .getObject("CurrentTrajectory")
            .setPoses(poses);
    }

    @Override
    public void execute() {
        // Get inputs
        double currentTime = timer.get();
        Pose2d currentPose = drive.getPose();

        // Get state
        HolonomicState desiredState = trajectory.sample(currentTime);

        if (headingOverride != null) {
            desiredState.withNewHolonomicAngle(headingOverride.get());
        }
        
        // Calculate speeds
        ChassisSpeeds targetChassisSpeeds = controller.calculate(currentPose, desiredState);

        // Apply speeds
        drive.runVelocity(targetChassisSpeeds);

        // Log inputs & outputs
        Logger.recordOutput("DrivePlannedPath/Timestamp", currentTime);

        Logger.recordOutput("DrivePlannedPath/Current Pose", currentPose);
        Logger.recordOutput("DrivePlannedPath/Desired Pose", desiredState.poseMeters());

        Logger.recordOutput("DrivePlannedPath/Current Angle", drive.getAngle());
        Logger.recordOutput("DrivePlannedPath/Desired Angle", desiredState.holonomicAngle());
        
        Logger.recordOutput("DrivePlannedPath/Current Velocity", drive.getRobotVelocity());
        Logger.recordOutput("DrivePlannedPath/Desired Velocity", targetChassisSpeeds);

        Logger.recordOutput("DrivePlannedPath/Drive Error", controller.getPoseError().getTranslation());
        Logger.recordOutput("DrivePlannedPath/Theta Error", controller.getRotationError());
    }

    @Override
    public boolean isFinished() {
        double totalTime = trajectory.getTotalTimeSeconds();

        boolean timeHasFinished = timer.hasElapsed(totalTime);
        boolean poseTolerance = controller.atReference();
        boolean tooLong = timer.hasElapsed(totalTime + 0.5);

        return (timeHasFinished && poseTolerance) || tooLong;
    }

    @Override
    public void end(boolean interrupted) {
        drive.getViz()
            .getObject("CurrentTrajectory")
            .setPoses();

        if (willStopAtEnd) {
            drive.stop();
        }
    }
}