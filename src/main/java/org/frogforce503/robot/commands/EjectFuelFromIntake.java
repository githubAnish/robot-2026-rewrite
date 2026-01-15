package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;

import edu.wpi.first.wpilibj2.command.Command;

public class EjectFuelFromIntake extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;

    public EjectFuelFromIntake(Superstructure superstructure) {
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();
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
