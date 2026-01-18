package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.rebuilt.ProximityUtil;
import org.frogforce503.lib.swerve.TeleopDriveController;
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Notes:
// involves some sort of fetching (no need of a separate fetching cmd, just needs to natural enough for the good driver, see orbit intake assist)
public class IntakeFuelFromGround extends Command {
    // Requirements
    private final Drive drive;
    private final Vision vision;

    private final Superstructure superstructure;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;

    private final BooleanSupplier autoAssistEnabled;

    // Constants
    private final double maxFetchingLinearSpeed = DriveConstants.maxLinearSpeed * 0.6; // TODO rough guess
    private final double maxFetchingLinearAcceleration = DriveConstants.maxLinearSpeed * 0.7; // TODO rough guess
    private final double lookaheadTimeSec = 0.15;

    // Controllers
    private final TeleopDriveController teleopController;
    private final PIDController assistController = new PIDController(1.0, 0.0, 0.0);

    public IntakeFuelFromGround(Drive drive, Vision vision, Superstructure superstructure, CommandXboxController xboxController, BooleanSupplier autoAssistEnabled) {
        this.drive = drive;
        this.vision = vision;

        this.superstructure = superstructure;
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();

        this.teleopController = new TeleopDriveController(drive, xboxController);

        this.autoAssistEnabled = autoAssistEnabled;

        addRequirements(drive, intakePivot, intakeRoller, indexer);
    }

    @Override
    public void initialize() {
        if (superstructure.isFull()) {
            return;
        }

        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
        indexer.setVelocity(IndexerConstants.INTAKE);
    }

    @Override
    public void execute() {
        // Get inputs
        Pose2d robotPose = drive.getLookaheadPose(lookaheadTimeSec);
        Pose2d targetPose = vision.getBestBallPose(); // TODO change in vision, currently it's in approximately middle of field, go to neutral zone outside of bump & trench

        // If compressed, back off indexer speed
        if (indexer.isCompressed()) {
            indexer.setVelocity(IndexerConstants.SLOW_MIX);
        } else {
            indexer.setVelocity(IndexerConstants.INTAKE);
        }

        // If fetching not wanted, skip fetching logic and do normal teleop drive
        if (!autoAssistEnabled.getAsBoolean() ||
            ProximityUtil.getDistanceBetweenPoses(robotPose, targetPose) < Units.inchesToMeters(40)
        ) {
            teleopController.update();
            return;
        }

        // Fetching logic (a basic driver-assist currently)
        // Get driver velocity
        Translation2d driverLinearVelocity = teleopController.getLinearVelocityFromJoysticks();
        double driverOmega = teleopController.getOmegaFromJoysticks();

        // Calculate speeds
        double xVelocity = driverLinearVelocity.getX() * DriveConstants.maxLinearSpeed;
        double yVelocity = driverLinearVelocity.getY() * DriveConstants.maxLinearSpeed;
        double omega = driverOmega * DriveConstants.maxOmega;

        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        // Apply assist
        double yError = robotPose.getTranslation().minus(targetPose.getTranslation()).getY();
        double yOutput = assistController.calculate(yError, 0);
        speeds.vyMetersPerSecond += yOutput;

        // Apply speeds
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                drive.getAngle()));

        // Log data
        Logger.recordOutput("IntakeFuelFromGround/Y Error", yError);
        Logger.recordOutput("IntakeFuelFromGround/Y Controller Output", yOutput);
    }

    @Override
    public boolean isFinished() {
        return superstructure.isFull();
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
        intakeRoller.stop();
        indexer.setVelocity(IndexerConstants.SLOW_MIX); // TODO maybe slow mixing in order to prep for shooting
    }
}
