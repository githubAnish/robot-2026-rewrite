package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexerWhenReady extends Command {
    private final Indexer indexer;

    private final BooleanSupplier isIntakingSupplier;
    private final BooleanSupplier isShootingSupplier;
    private final BooleanSupplier isShotFeasibleSupplier;

    public RunIndexerWhenReady(
        Indexer indexer,
        BooleanSupplier isIntakingSupplier,
        BooleanSupplier isShootingSupplier,
        BooleanSupplier isShotFeasibleSupplier
    ) {
        this.indexer = indexer;
        
        this.isIntakingSupplier = isIntakingSupplier;
        this.isShootingSupplier = isShootingSupplier;
        this.isShotFeasibleSupplier = isShotFeasibleSupplier;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (isShootingSupplier.getAsBoolean() && isShotFeasibleSupplier.getAsBoolean()) { // Run indexer only if shot feasible to prevent jams with feeder
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