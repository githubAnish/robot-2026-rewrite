package org.frogforce503.robot.auto;

import java.io.File;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

@SuppressWarnings("unused")
public class WarmupExecutor {
    private final Drive drive;
    private final FollowPath.Builder blineAutoBuilder;

    private final String blinePathSuffix = ".json";

    public WarmupExecutor(Drive drive, FollowPath.Builder blineAutoBuilder) {
        this.drive = drive;
        this.blineAutoBuilder = blineAutoBuilder;
    }

    public void periodic() {
        warmupShotCalculator();
        warmupBLinePaths();
    }

    // Main methods
    private void warmupShotCalculator() {
        ShotCalculator.getInstance().calculateShotInfo(
            drive.getPose(),
            drive.getRobotVelocity(),
            drive.getFieldVelocity()
        );
    }

    private void warmupBLinePaths() {
        String blineDir = Filesystem.getDeployDirectory().getAbsolutePath() + "/autos/paths";
        File[] files = new File(blineDir).listFiles((dir, name) -> name.endsWith(blinePathSuffix));

        if (files == null) {
            throw new RuntimeException("Failed to warmup BLine paths");
        }

        for (File file : files) {
            Path traj = new Path(stripExtension(file.getName(), blinePathSuffix));
            blineAutoBuilder.build(traj);
        }
    }
    
    // Utility methods
    private void printWarmupTime(Runnable action) { // Wrap over another method to find warmup time
        long startTime = System.nanoTime();
        action.run();
        long endTime = System.nanoTime();
        System.out.println("Warmup took " + (endTime - startTime) / 1e9 + " s");
    }

    private String stripExtension(String fileName, String extension) {
        return fileName.substring(0, fileName.length() - extension.length());
    }
}