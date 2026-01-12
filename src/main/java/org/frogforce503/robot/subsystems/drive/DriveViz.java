package org.frogforce503.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.frogforce503.robot.FieldInfo;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class DriveViz {
    // Constants
    private final boolean logModules = true;
    private final double maxSpeed = DriveConstants.maxLinearSpeed;

    // Module viz
    private final LoggedMechanism2d[] moduleMechanisms;
    private final LoggedMechanismLigament2d[] moduleSpeeds;
    private final LoggedMechanismLigament2d[] moduleDirections;

    public DriveViz() {
        moduleMechanisms =
            new LoggedMechanism2d[] {
                new LoggedMechanism2d(1, 1),
                new LoggedMechanism2d(1, 1),
                new LoggedMechanism2d(1, 1),
                new LoggedMechanism2d(1, 1)
            };

        moduleSpeeds =
            new LoggedMechanismLigament2d[] {
                createModuleSpeedLigament(0),
                createModuleSpeedLigament(1),
                createModuleSpeedLigament(2),
                createModuleSpeedLigament(3),
            };

        moduleDirections =
            new LoggedMechanismLigament2d[] {
                createModuleDirectionLigament(0),
                createModuleDirectionLigament(1),
                createModuleDirectionLigament(2),
                createModuleDirectionLigament(3),
            };
    }

    private LoggedMechanismLigament2d createModuleSpeedLigament(int moduleIndex) {
        return
            moduleMechanisms[moduleIndex]
                .getRoot("RootSpeed", 0.5, 0.5)
                .append(new LoggedMechanismLigament2d("Speed", 0.5, 0));
    }

    private LoggedMechanismLigament2d createModuleDirectionLigament(int moduleIndex) {
        return
            moduleMechanisms[moduleIndex]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite)));
    }

    public void update(SwerveDriveState state) {
        if (state == null || state.Pose == null || state.ModuleStates == null) {
            return;
        }

        // Get inputs
        Pose2d pose = state.Pose;
        ChassisSpeeds robotRelativeSpeeds = state.Speeds;
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());

        // Log robot pose
        Logger.recordOutput("Drive/Pose", pose);
        FieldInfo.setRobotPose(pose);

        // Log robot velocities
        Translation2d robotRelativeVelocity = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);

        Logger.recordOutput("Drive/Speed", robotRelativeVelocity.getNorm());
        Logger.recordOutput("Drive/Velocity/RobotVelocityX", robotRelativeSpeeds.vxMetersPerSecond);
        Logger.recordOutput("Drive/Velocity/RobotVelocityY", robotRelativeSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/Velocity/FieldVelocityX", fieldRelativeSpeeds.vxMetersPerSecond);
        Logger.recordOutput("Drive/Velocity/FieldVelocityY", fieldRelativeSpeeds.vyMetersPerSecond);

        // Log modules
        if (logModules) {
            Logger.recordOutput("Drive/Modules/ModuleStates", state.ModuleStates);

            for (int i = 0; i < 4; ++i) {
                moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
                moduleDirections[i].setAngle(state.ModuleStates[i].angle);
                moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));
            }

            Logger.recordOutput("Drive/Modules/Viz/FrontLeft", moduleMechanisms[0]);
            Logger.recordOutput("Drive/Modules/Viz/FrontRight", moduleMechanisms[1]);
            Logger.recordOutput("Drive/Modules/Viz/BackLeft", moduleMechanisms[2]);
            Logger.recordOutput("Drive/Modules/Viz/BackRight", moduleMechanisms[3]);
        }

        // Log other important info
        Logger.recordOutput("Drive/OdomPeriod", state.OdometryPeriod);
    }
}