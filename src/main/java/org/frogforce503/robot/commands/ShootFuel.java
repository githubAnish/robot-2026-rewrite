package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.indexer.Indexer;
import org.frogforce503.robot.subsystems.indexer.IndexerConstants;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator;
import org.frogforce503.robot.viz.GameViz;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootFuel extends Command {
    private final Indexer indexer;
    private final GameViz gameViz;

    public ShootFuel(Indexer indexer, GameViz gameViz) {
        this.indexer = indexer;
        this.gameViz = gameViz;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        boolean isShotFeasible = LaunchCalculator.getInstance().isShotFeasible();

        // Run indexer if shot feasible
        indexer.setVelocity(isShotFeasible ? IndexerConstants.SHOOT : 0.0);

        // Simulate shooting
        if (RobotBase.isSimulation() && isShotFeasible) {
            gameViz.shootFuel();
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