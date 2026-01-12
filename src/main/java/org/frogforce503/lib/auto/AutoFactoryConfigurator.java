package org.frogforce503.lib.auto;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public final class AutoFactoryConfigurator {
    private AutoFactoryConfigurator() {}

    public static void configurePathPlanner(Drive drive) {
        final PIDConfig linearPID = Robot.bot.getFollowerLinearPID();
        final PIDConfig thetaPID = Robot.bot.getFollowerThetaPID();

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

        } catch (Exception e) {
            System.out.println("Failed to load PathPlanner config and configure AutoBuilder" + ErrorUtil.attachJavaClassName(AutoFactoryConfigurator.class));
            e.printStackTrace();
        }
    }

    public static AutoFactory configureChoreo(Drive drive) {
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