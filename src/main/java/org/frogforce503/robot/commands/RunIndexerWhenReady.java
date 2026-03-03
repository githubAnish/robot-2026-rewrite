package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexerWhenReady extends Command {
    private final Indexer indexer;
    private final BooleanSupplier isIntakingSupplier;
    private final BooleanSupplier isShootingSupplier;

    public RunIndexerWhenReady(Indexer indexer, BooleanSupplier isIntakingSupplier, BooleanSupplier isShootingSupplier) {
        this.indexer = indexer;
        this.isIntakingSupplier = isIntakingSupplier;
        this.isShootingSupplier = isShootingSupplier;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (isShootingSupplier.getAsBoolean()) {
            indexer.setVelocity(IndexerConstants.SHOOT);
        } else if (isIntakingSupplier.getAsBoolean()) {
            indexer.setVelocity(IndexerConstants.INTAKE);
        } else {
            indexer.setVelocity(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }
}