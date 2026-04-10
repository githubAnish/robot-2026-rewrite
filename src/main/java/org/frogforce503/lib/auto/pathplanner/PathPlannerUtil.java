package org.frogforce503.lib.auto.pathplanner;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class PathPlannerUtil {
    private PathPlannerUtil() {}

    public static void configureAutoBuilder(Drive drive) {
        final PIDConfig linearPID = DriveConstants.pathplannerLinearPID;
        final PIDConfig thetaPID = DriveConstants.pathplannerThetaPID;

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
                    new PIDConstants(linearPID.kP(), linearPID.kI(), linearPID.kD()),
                    new PIDConstants(thetaPID.kP(), thetaPID.kI(), thetaPID.kD())
                ),
                config,
                FieldConstants::isRed,
                drive);

        } catch (IOException | ParseException e) {
            System.out.println("Failed to load PathPlanner config and configure AutoBuilder" + ErrorUtil.attachJavaClassName(PathPlannerUtil.class));
            e.printStackTrace();
        }
    }

    public static PathPlannerPath loadTrajectory(String name) {
        try {
            return PathPlannerPath.fromPathFile(name);
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("Error loading path " + name + ErrorUtil.attachJavaClassName(PathPlannerUtil.class));
            e.printStackTrace();
            return null;
        }
    }

    public static Command createOTFPath(Pose2d robotPose, Pose2d... poseWaypoints) {
        List<Pose2d> poses = new ArrayList<>(List.of(robotPose));
        Collections.addAll(poses, poseWaypoints);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            DriveConstants.pathplannerConstraints,
            null,
            new GoalEndState(0.0, poseWaypoints[poseWaypoints.length - 1].getRotation())
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    public static Pose2d[] getPoses(PathPlannerPath... paths) {
        return
            Arrays
                .stream(paths)
                .flatMap(traj -> traj.getPathPoses().stream())
                .toArray(Pose2d[]::new);
    }
}