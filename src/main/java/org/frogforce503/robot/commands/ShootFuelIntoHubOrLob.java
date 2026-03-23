package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootFuelIntoHubOrLob extends Command {
    private final Feeder feeder;
    private final GameViz gameViz;
    private final BooleanSupplier isShotFeasibleSupplier;

    public ShootFuelIntoHubOrLob(Feeder feeder, GameViz gameViz, BooleanSupplier isShotFeasibleSupplier) {
        this.feeder = feeder;
        this.gameViz = gameViz;

        this.isShotFeasibleSupplier = isShotFeasibleSupplier;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        boolean isShotFeasible = isShotFeasibleSupplier.getAsBoolean();

        // Run feeder if shot feasible (flywheels already run in TrackTargetCommand)
        if (isShotFeasible) {
            feeder.setVelocity(FeederConstants.SHOOT);
        }

        // Simulate shooting
        if (RobotBase.isSimulation() && isShotFeasible) {
            gameViz.shootFuel(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}