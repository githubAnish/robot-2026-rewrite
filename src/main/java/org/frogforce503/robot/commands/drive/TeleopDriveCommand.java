package org.frogforce503.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

import java.util.Optional;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(JoystickUtil.class)
public class TeleopDriveCommand extends Command {
    // Requirements
    private final Drive drive;
    private final CommandXboxController xboxController;

    // Constants
    private final PIDController headingHoldController = new PIDController(5, 0, 0);
    private final double headingHoldTolerance = Units.degreesToRadians(1.0);
    private final double kHeadingHoldDelay = 0.25;
    private final Timer headingHoldTimer = new Timer();

    private static final double kOmegaDeadband = 0.05;
    private static final double kTranslationDeadband = 0.02;

    // State
    private TeleopDriveState currentState = TeleopDriveState.IDLE;
    private Optional<Rotation2d> headingSetpoint = Optional.empty();
    private boolean robotRelative = false;
    private boolean slowMode = false;

    private enum TeleopDriveState {
        FIELD_RELATIVE,
        FIELD_RELATIVE_HEADING_HOLD,
        ROBOT_RELATIVE,
        IDLE
    }

    public TeleopDriveCommand(Drive drive, CommandXboxController xboxController) {
        this.drive = drive;
        this.xboxController = xboxController;

        headingHoldController.enableContinuousInput(-Math.PI, Math.PI);
        headingHoldController.setTolerance(headingHoldTolerance);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        headingHoldTimer.reset();
    }

    @Override
    public void execute() {
        // Get driver input velocities
        Translation2d driverLinearVelocity = xboxController.getLinearVelocityFromJoysticks();
        double driverOmega = xboxController.getOmegaFromJoysticks();

        // Apply slow mode if necessary
        double translationScalar = slowMode ? 0.25 : 1.0;
        double rotationScalar = slowMode ? 0.25 : 1.0;

        // Calculate speeds
        double xVelocity = driverLinearVelocity.getX() * DriveConstants.maxLinearSpeed * translationScalar;
        double yVelocity = driverLinearVelocity.getY() * DriveConstants.maxLinearSpeed * translationScalar;
        double omega = driverOmega * DriveConstants.maxOmega * rotationScalar;

        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        // Apply speeds to drivetrain
        boolean isTranslating = Math.hypot(xVelocity, yVelocity) > kTranslationDeadband;
        boolean isRotating = Math.abs(omega) > kOmegaDeadband;

        if (robotRelative) {
            currentState = TeleopDriveState.ROBOT_RELATIVE;
        } else if (isRotating) {
            currentState = TeleopDriveState.FIELD_RELATIVE;
        } else if (isTranslating) {
            currentState = headingSetpoint.isPresent() ? TeleopDriveState.FIELD_RELATIVE_HEADING_HOLD : TeleopDriveState.FIELD_RELATIVE;
        } else {
            currentState = TeleopDriveState.IDLE;
        }

        switch (currentState) {
            case FIELD_RELATIVE:
                lockHeadingIfRotationStopped(isRotating);
                runFieldRelativeVelocity(speeds);
                break;

            case FIELD_RELATIVE_HEADING_HOLD:
                double omegaCorrection = headingHoldController.calculate(drive.getAngle().getRadians(), headingSetpoint.get().getRadians());
                speeds.omegaRadiansPerSecond = MathUtil.clamp(omegaCorrection, -DriveConstants.maxOmega, DriveConstants.maxOmega);

                runFieldRelativeVelocity(speeds);
                break;

            case ROBOT_RELATIVE:
                drive.runVelocity(speeds);
                break;

            case IDLE:
                drive.stop();
                break;
        }

        // Log data
        Logger.recordOutput("TeleopDriveCommand/State", currentState);
        Logger.recordOutput("TeleopDriveCommand/SlowModeEnabled", slowMode);
        Logger.recordOutput("TeleopDriveCommand/RobotRelative", robotRelative);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public void toggleRobotRelative() {
        robotRelative = !robotRelative;
    }

    public void runFieldRelativeVelocity(ChassisSpeeds speeds) {
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                FieldConstants.isRed()
                    ? drive.getAngle().plus(Rotation2d.kPi)
                    : drive.getAngle()));
    }

    private void lockHeadingIfRotationStopped(boolean isRotating) {
        if (isRotating) {
            headingHoldTimer.reset();
            headingSetpoint = Optional.empty();
        } else {
            headingHoldTimer.start();
            if (headingHoldTimer.hasElapsed(kHeadingHoldDelay)) {
                headingSetpoint = Optional.of(drive.getAngle());
            }
        }
    }
}