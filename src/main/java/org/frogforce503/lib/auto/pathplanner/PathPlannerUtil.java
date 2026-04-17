package org.frogforce503.lib.auto.pathplanner;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class PathPlannerUtil {
    private PathPlannerUtil() {}

    public static void configureAutoBuilder(Drive drive) {
        try {
            var config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                drive::getPose,
                drive::setPose,
                drive::getRobotVelocity,
                (speeds, feedforwards) -> {
                    drive.runVelocity(
                        speeds,
                        feedforwards.robotRelativeForcesXNewtons(),
                        feedforwards.robotRelativeForcesYNewtons());
                },
                new PPHolonomicDriveController(
                    DriveConstants.pathplannerLinearPID.toPathPlannerConstraints(),
                    DriveConstants.pathplannerThetaPID.toPathPlannerConstraints()
                ),
                config,
                FieldConstants::isRed,
                drive);

        } catch (IOException | ParseException e) {
            System.out.println("Failed to load PathPlanner config and configure AutoBuilder.");
            e.printStackTrace();
        }
    }

    public static Pose2d[] getPoses(PathPlannerPath... paths) {
        return
            Arrays
                .stream(paths)
                .flatMap(traj -> traj.getPathPoses().stream())
                .toArray(Pose2d[]::new);
    }

    public static PathPlannerPath loadTrajectory(String name) {
        try {
            return PathPlannerPath.fromPathFile(name);
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("Error loading path " + name + ".");
            e.printStackTrace();
            return null;
        }
    }

    public static Command createOTFPath(Pose2d robotPose, Pose2d... waypoints) {
        List<Pose2d> poses = new ArrayList<>();
        
        poses.add(robotPose);

        for (Pose2d pose : waypoints) {
            poses.add(pose);
        }

        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(poses),
            DriveConstants.pathplannerConstraints,
            null,
            new GoalEndState(0.0, waypoints[waypoints.length - 1].getRotation())
        );

        // Prevent path from being flipped if coordinates already correct
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
}