package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.lib.auto.planned_path.PlannedPath.HolonomicState;
import org.frogforce503.lib.auto.planned_path.PlannedPathFactory;
import org.frogforce503.lib.auto.planned_path.components.Waypoint;
import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.lib.swerve.SwervePathController;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(JoystickUtil.class)
public class IntakeFuelFromOutpost extends Command {
    private final Drive drive;
    private final Vision vision;

    private final IntakeRoller intakeRoller;
    private final Indexer indexer;

    private final CommandXboxController xboxController;
    private final BooleanSupplier autoAssistEnabled;

    // Constants
    private final double kLookaheadTimeSec = 0.05;
    private final double trajectoryMaxVelocityMetersPerSec = DriveConstants.maxLinearSpeed * 0.7;
    private final double trajectoryMaxAccelerationMetersPerSec2 = DriveConstants.maxLinearSpeed * 0.7;

    // Trajectory
    private final SwervePathController trajectoryController = DriveConstants.pathFollower;
    private final Timer trajectoryTimer = new Timer();
    private PlannedPath trajectory;

    public IntakeFuelFromOutpost(
        Drive drive,
        Vision vision,
        Superstructure superstructure,
        CommandXboxController xboxController,
        BooleanSupplier autoAssistEnabled
    ) {
        this.drive = drive;
        this.vision = vision;

        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();

        this.xboxController = xboxController;
        this.autoAssistEnabled = autoAssistEnabled;

        addRequirements(drive, intakeRoller, indexer);
    }

    @Override
    public void initialize() {
        if (indexer.isCompressed()) {
            return;
        }

        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
        indexer.setVelocity(IndexerConstants.INTAKE);

        vision.setDesiredAprilTagGoal(AprilTagGoal.OUTPOST_ALIGNMENT);

        Pose2d robotPose = drive.getLookaheadPose(kLookaheadTimeSec);

        double linearVelocity =
            Math.hypot(
                drive.getRobotVelocity().vxMetersPerSecond,
                drive.getRobotVelocity().vyMetersPerSecond);

        Pose2d targetPose =
            FieldConstants.isRed()
                ? FieldConstants.Outpost.red
                : FieldConstants.Outpost.blue;

        trajectory =
            PlannedPathFactory.generate(
                trajectoryMaxVelocityMetersPerSec,
                trajectoryMaxAccelerationMetersPerSec2,
                linearVelocity,
                0,
                Waypoint.fromHolonomicPose(robotPose),
                Waypoint.fromHolonomicPose(targetPose));

        trajectoryController.reset();
        trajectoryTimer.restart();
    }

    @Override
    public void execute() {
        // Get inputs
        Pose2d robotPose = drive.getLookaheadPose(kLookaheadTimeSec);

        // Calculate default teleop velocities
        Translation2d driverLinearVelocity = xboxController.getLinearVelocityFromJoysticks();
        double driverOmega = xboxController.getOmegaFromJoysticks();

        double xVelocity = driverLinearVelocity.getX() * DriveConstants.maxLinearSpeed;
        double yVelocity = driverLinearVelocity.getY() * DriveConstants.maxLinearSpeed;
        double omega = driverOmega * DriveConstants.maxOmega;

        ChassisSpeeds speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                omega,
                drive.getAngle());

        // Normal teleop drive if auto assist not wanted
        if (!autoAssistEnabled.getAsBoolean()) {
            drive.runVelocity(speeds);
            return;
        }

        double currentTime = trajectoryTimer.get();
        HolonomicState desiredState = trajectory.sample(currentTime);
        speeds = trajectoryController.calculate(robotPose, desiredState);

        // Apply speeds
        drive.runVelocity(speeds);

        // Log data
        Logger.recordOutput("IntakeFuelFromOutpost/Timestamp", currentTime);
        Logger.recordOutput("IntakeFuelFromOutpost/Drive Error", trajectoryController.getPoseError().getTranslation());
        Logger.recordOutput("IntakeFuelFromOutpost/Theta Error", trajectoryController.getRotationError());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        trajectoryTimer.stop();

        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);

        intakeRoller.stop();
        indexer.setVelocity(IndexerConstants.SLOW_MIX); // TODO maybe slow mixing in order to prep for shooting
    }
}
