package org.frogforce503.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.frogforce503.lib.swerve.TeleopDriveController;
import org.frogforce503.robot.subsystems.drive.Drive;

public class TeleopDriveCommand extends Command {
    // Requirements
    private final Drive drive;
    private final TeleopDriveController teleopController;

    public TeleopDriveCommand(Drive drive, CommandXboxController xboxController) {
        this.drive = drive;
        this.teleopController = new TeleopDriveController(drive, xboxController);

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Apply slow mode if necessary
        teleopController.setTeleopTranslationScalar(drive.isSlowMode() ? 0.25 : 1.0);
        teleopController.setTeleopRotationScalar(drive.isSlowMode() ? 0.25 : 1.0);

        // Update controller
        teleopController.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}