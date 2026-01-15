package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;

import edu.wpi.first.wpilibj2.command.Command;

// similar to 2022 and 2024, have a series of steps
public class ClimbSequence extends Command {
    private final Drive drive;
    private final Vision vision;
    private final Climber climber;

    private final BooleanSupplier triggerPressed;

    // not really accurate, but some rough idea
    private enum ClimbState {
        RAISE_FOR_L1,
        STOW_AT_L1,
        RAISE_FOR_L2,
        STOW_AT_L2,
        RAISE_FOR_L3,
        STOW_AT_L3,
        FINISHED,
    }
    
    public ClimbSequence(Drive drive, Vision vision, Superstructure superstructure, Climber climber, BooleanSupplier triggerPressed) {
        this.drive = drive;
        this.vision = vision;
        this.climber = climber;

        this.triggerPressed = triggerPressed;

        addRequirements(climber);
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
