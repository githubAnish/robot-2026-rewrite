package org.frogforce503.lib.auto.pathplanner;

import java.io.IOException;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public final class PathPlannerUtil {
    private PathPlannerUtil() {}

    public static void configureAutoBuilder(Drive drive) {
        final PIDConfig linearPID = DriveConstants.linearPID;
        final PIDConfig thetaPID = DriveConstants.thetaPID;

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
                () -> Constants.useAllianceFlipping,
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
            System.out.println("Error creating auto" + ErrorUtil.attachJavaClassName(PathPlannerUtil.class));
            e.printStackTrace();
            return null;
        }
    }

    public static PathPlannerPath loadChoreoTrajectory(String name) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(name);
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("Error creating auto" + ErrorUtil.attachJavaClassName(PathPlannerUtil.class));
            e.printStackTrace();
            return null;
        }
    }
}