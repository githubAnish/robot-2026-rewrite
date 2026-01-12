package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFuelFromOutpost extends Command {
    private final Drive drive;
    private final Vision vision;

    private final BooleanSupplier autoAssistEnabled;

    public IntakeFuelFromOutpost(Drive drive, Vision vision, Superstructure superstructure, BooleanSupplier autoAssistEnabled) {
        this.drive = drive;
        this.vision = vision;
        this.autoAssistEnabled = autoAssistEnabled;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
