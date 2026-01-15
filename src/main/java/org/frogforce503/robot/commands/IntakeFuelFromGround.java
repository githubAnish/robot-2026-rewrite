package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.lib.auto.planned_path.PlannedPath.HolonomicState;
import org.frogforce503.lib.auto.planned_path.PlannedPathFactory;
import org.frogforce503.lib.auto.planned_path.components.Waypoint;
import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.lib.swerve.SwervePathController;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

// Notes:
// involves some sort of fetching (no need of a separate fetching cmd, just needs to natural enough for the good driver, see orbit intake assist)
@ExtensionMethod({JoystickUtil.class})
public class IntakeFuelFromGround extends Command {
    // Requirements
    private final Drive drive;
    private final Vision vision;

    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;

    private final CommandXboxController xboxController;

    private final BooleanSupplier autoAssistEnabled;

    // Constants
    private final double maxFetchingLinearSpeed = DriveConstants.maxLinearSpeed * 0.6; // TODO rough guess
    private final double maxFetchingLinearAcceleration = DriveConstants.maxLinearSpeed * 0.7; // TODO rough guess
    private final double lookaheadTimeSec = 0.15;

    // Trajectory
    private final SwervePathController trajectoryController = DriveConstants.pathFollower;
    private final Timer trajectoryTimer = new Timer();
    private PlannedPath trajectory;

    public IntakeFuelFromGround(Drive drive, Vision vision, Superstructure superstructure, CommandXboxController xboxController, BooleanSupplier autoAssistEnabled) {
        this.drive = drive;
        this.vision = vision;

        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();

        this.xboxController = xboxController;

        this.autoAssistEnabled = autoAssistEnabled;

        addRequirements(drive, intakePivot, intakeRoller, indexer);
    }

    @Override
    public void initialize() {
        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
        indexer.setVelocity(IndexerConstants.INTAKE);

        // Initialize path to fuel
        Pose2d robotPose = drive.getFuturePose(lookaheadTimeSec);
        Pose2d targetPose = new Pose2d(3,3,new Rotation2d()); // TODO get from vision

        double linearVelocity =
            Math.hypot(
                drive.getRobotVelocity().vxMetersPerSecond,
                drive.getRobotVelocity().vyMetersPerSecond);

        trajectory =
            PlannedPathFactory.generate(
                maxFetchingLinearSpeed,
                maxFetchingLinearAcceleration,
                linearVelocity,
                0,
                Waypoint.fromHolonomicPose(robotPose),
                Waypoint.fromHolonomicPose(targetPose));

        trajectoryController.reset();
        trajectoryTimer.restart();
    }

    @Override
    public void execute() {
        // If compressed, back off indexer speed
        if (indexer.isCompressed()) {
            indexer.setVelocity(IndexerConstants.SLOW_MIX);
        } else {
            indexer.setVelocity(IndexerConstants.INTAKE);
        }

        // If fetching not wanted, skip fetching logic
        if (!autoAssistEnabled.getAsBoolean()) {
            return;
        }

        // Fetching logic (TODO currently not an driver-assist, just following a on-the-fly trajectory)
        double currentTime = trajectoryTimer.get();
        HolonomicState desiredState = trajectory.sample(currentTime);
        ChassisSpeeds targetChassisSpeeds = trajectoryController.calculate(drive.getPose(), desiredState);
        drive.runVelocity(targetChassisSpeeds);

        // Log data
        Logger.recordOutput("IntakeFuelFromGround/Timestamp", currentTime);
        Logger.recordOutput("IntakeFuelFromGround/Drive Error", trajectoryController.getPoseError().getTranslation());
        Logger.recordOutput("IntakeFuelFromGround/Theta Error", trajectoryController.getRotationError());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
        intakeRoller.stop();
        indexer.setVelocity(IndexerConstants.SLOW_MIX); // TODO maybe slow mixing in order to prep for shooting
    }
}
