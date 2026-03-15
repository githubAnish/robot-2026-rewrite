package org.frogforce503.robot.auto;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Stream;

import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@SuppressWarnings("unused")
public class WarmupExecutor {
    private final Drive drive;
    
    private final String choreoPathSuffix = ".traj";
    private final String pathPlannerPathSuffix = ".path";

    public WarmupExecutor(Drive drive) {
        this.drive = drive;

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

    public void periodic() {
        warmupPathPlannerPaths();
        warmupChoreoPaths();
        warmupDrive();
        warmupShotCalculator();
    }

    // Main methods
    private void warmupDrive() {
        DriveConstants.pathFollower.calculate(
            drive.getPose(),
            Pose2d.kZero,
            0.1,
            0.1,
            0.1);
    }

    private void warmupShotCalculator() {
        ShotCalculator.calculateShotInfo(
            drive.getPose(),
            drive.getRobotVelocity(),
            drive.getFieldVelocity()
        );
    }

    private void warmupPathPlannerPaths() {
        Path pathPlannerPathsDir =
            Path.of(
                Filesystem.getDeployDirectory().getAbsolutePath(),
                "pathplanner",
                "paths");

        try (Stream<Path> paths = Files.list(pathPlannerPathsDir)) {
            paths
                .filter(path -> path.getFileName().toString().endsWith(pathPlannerPathSuffix))
                .map(path -> stripExtension(path, pathPlannerPathSuffix))
                .forEach(name -> {
                    PathPlannerUtil.loadTrajectory(name);
                });

        } catch (IOException e) {
            throw new RuntimeException("Failed to warmup PathPlanner paths", e);
        }
    }

    private void warmupChoreoPaths() {
        Path choreoDir =
            Path.of(
                Filesystem.getDeployDirectory().getAbsolutePath(),
                "choreo");

        try (Stream<Path> paths = Files.list(choreoDir)) {
            paths
                .filter(path -> path.getFileName().toString().endsWith(choreoPathSuffix))
                .map(path -> stripExtension(path, choreoPathSuffix))
                .forEach(name -> {
                    PathPlannerUtil.loadChoreoTrajectory(name);
                });

        } catch (IOException e) {
            throw new RuntimeException("Failed to warmup choreo paths", e);
        }
    }
    
    // Utility methods
    private void printWarmupTime(Runnable action) { // Wrap over another method to find warmup time
        long startTime = System.nanoTime();
        action.run();
        long endTime = System.nanoTime();
        System.out.println("Warmup took " + (endTime - startTime) / 1e9 + " s");
    }

    private String stripExtension(Path path, String extension) {
        String fileName = path.getFileName().toString();
        return fileName.substring(0, fileName.length() - extension.length());
    }
}