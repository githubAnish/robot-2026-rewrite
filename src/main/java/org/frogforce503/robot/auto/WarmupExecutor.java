package org.frogforce503.robot.auto;

import java.io.File;

import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public final class WarmupExecutor {
    private final Drive drive;
    private final FollowPath.Builder blineAutoBuilder;

    private final String pathPlannerPathSuffix = ".path";
    private final String blinePathSuffix = ".json";

    private final int maxWarmupCycles = 50; // ~1 sec at 20ms loop
    private int warmupCycles = 0;

    public WarmupExecutor(Drive drive, FollowPath.Builder blineAutoBuilder) {
        this.drive = drive;
        this.blineAutoBuilder = blineAutoBuilder;
    }

    public void initialize() {
        // Warmup PathPlanner cmds
        CommandScheduler.getInstance().schedule(
            FollowPathCommand
                .warmupCommand()
                .withName("FollowPathCommand Warmup")
                .ignoringDisable(true),
                
            PathfindingCommand
                .warmupCommand()
                .withName("PathfindingCommand Warmup")
                .ignoringDisable(true));

        // Load paths
        printWarmupTime("PathPlanner warmup", this::warmupPathPlannerPaths);
        printWarmupTime("BLine warmup", this::warmupBLinePaths);
    }

    public void update() {
        if (warmupCycles < maxWarmupCycles) {
            warmupShotCalculator();
            warmupCycles++;
        }
    }

    private void warmupShotCalculator() {
        LaunchCalculator.getInstance().calculateShotInfo(
            drive.getPose(),
            drive.getRobotVelocity(),
            drive.getFieldVelocity()
        );
    }

    private void warmupPathPlannerPaths() {
        String pathPlannerDir = Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner/paths";
        File[] files = new File(pathPlannerDir).listFiles((dir, name) -> name.endsWith(pathPlannerPathSuffix));

        if (files == null) {
            System.err.println("Failed to warmup PathPlanner paths");
            return;
        }

        for (File file : files) {
            PathPlannerUtil.loadTrajectory(stripExtension(file.getName(), pathPlannerPathSuffix));
        }
    }

    private void warmupBLinePaths() {
        String blineDir = Filesystem.getDeployDirectory().getAbsolutePath() + "/autos/paths";
        File[] files = new File(blineDir).listFiles((dir, name) -> name.endsWith(blinePathSuffix));

        if (files == null) {
            System.err.println("Failed to warmup BLine paths");
            return;
        }

        for (File file : files) {
            Path traj = new Path(stripExtension(file.getName(), blinePathSuffix));
            blineAutoBuilder.build(traj);
        }
    }
    
    // Utility methods
    private void printWarmupTime(String key, Runnable action) { // Wrap over another method to find warmup time
        long startTime = System.nanoTime();
        action.run();
        long endTime = System.nanoTime();
        System.out.println(key + " took " + (endTime - startTime) / 1e9 + " seconds.");
    }

    private String stripExtension(String fileName, String extension) {
        return fileName.substring(0, fileName.length() - extension.length());
    }
}