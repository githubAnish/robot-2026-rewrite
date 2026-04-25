package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.feeder.Feeder;
import org.frogforce503.robot.subsystems.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.indexer.Indexer;
import org.frogforce503.robot.subsystems.indexer.IndexerConstants;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRollerConstants;

import edu.wpi.first.wpilibj2.command.Command;

/** Runs the intake, indexer, and feeder in the opposite direction to unjam any stuck fuel. */
public class EjectFuelFromIntake extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;
    private final Feeder feeder;

    public EjectFuelFromIntake(
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Indexer indexer,
        Feeder feeder
    ) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.indexer = indexer;
        this.feeder = feeder;

        addRequirements(intakePivot, intakeRoller, indexer, feeder);
    }

    @Override
    public void initialize() {
        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.EJECT);
        indexer.setVelocity(IndexerConstants.EJECT);
        feeder.setVelocity(FeederConstants.EJECT_FROM_INTAKE);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
        intakeRoller.stop();
        indexer.stop();
        feeder.stop();
    }
}
