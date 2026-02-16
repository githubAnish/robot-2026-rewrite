package org.frogforce503.robot.auto;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.lib.auto.planned_path.PlannedPathFactory;
import org.frogforce503.lib.auto.planned_path.components.Waypoint;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class WarmupExecutor {
    private final Drive drive;
    private final AutoChooser autoChooser;

    public WarmupExecutor(Drive drive, AutoChooser autoChooser) {
        this.drive = drive;
        this.autoChooser = autoChooser;

        // Warmup PathPlanner cmds on robot init
        CommandScheduler.getInstance().schedule(
            FollowPathCommand
                .warmupCommand()
                .withName("FollowPathCommand Warmup")
                .ignoringDisable(true),
                
            PathfindingCommand
                .warmupCommand()
                .withName("PathfindingCommand Warmup")
                .ignoringDisable(true));
    }

    public void disabledInit() {
        // NetworkTableInstance.getDefault().flush();
        // System.gc();
    }

    public void disabledPeriodic() {
        warmupPlannedPathGenerator();
        warmupPaths();
        warmupDrive();
        warmupShotCalculator();
    }

    /** Wrap this method over another method to find its warmup time. */
    private void printWarmupTime(Runnable action) {
        long startTime = System.nanoTime();
        action.run();
        long endTime = System.nanoTime();
        System.out.println("Warmup took " + (endTime - startTime)/1e9 + " s");
    }

    /** Warmups the {@link PlannedPathFactory} to speed up {@link PlannedPath} generation. */
    private void warmupPlannedPathGenerator() {
        // Random values inserted in for path constraints & waypoints
        PlannedPathFactory
            .generate(
                Units.inchesToMeters(13),
                Units.inchesToMeters(19),
                3.0,
                0.7,
                Waypoint.fromHolonomicPose(new Pose2d(0, 0, new Rotation2d(Math.PI/4))),
                Waypoint.fromHolonomicPose(new Pose2d(2.5, 2.5, new Rotation2d(-Math.PI/3))),
                Waypoint.fromHolonomicPose(new Pose2d(10, 7.5, new Rotation2d(Math.PI))));
    }

    private void warmupPaths() {
        
    }

    private void warmupDrive() {
        // Warmup path follower
        DriveConstants.pathFollower.calculate(
            drive.getPose(),
            Pose2d.kZero,
            0.1,
            0.1,
            0.1);
    }

    private void warmupShotCalculator() {
        ShotCalculator.calculateHubShotInfo(
            drive.getPose(),
            drive.getRobotVelocity(),
            drive.getFieldVelocity()
        );

        ShotCalculator.calculateTurretRobotRelativeSetpoint(
            Rotation2d.kZero,
            3,
            Rotation2d.kZero,
            3
        );
    }
}