package org.frogforce503.robot.commands;

import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootFuelIntoHubOrLob extends Command {
    private final Drive drive;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    private final GameViz gameViz;

    public ShootFuelIntoHubOrLob(
        Drive drive,
        Indexer indexer,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.indexer = indexer;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;

        addRequirements(feeder, hood, flywheels);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Define shot parameters
        double hoodAngleRad = 0.0;
        double hoodVelocityRadPerSec = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;

        // Get latest shot info
        ShotInfo shotInfo =
            ShotCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        switch (ShotCalculator.getInstance().getShotPreset()) {
            case NONE:
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                break;

            case BATTER:
                hoodAngleRad = HoodConstants.BATTER;
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                break;

            case TRENCH:
                hoodAngleRad = HoodConstants.TRENCH;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TRENCH;
                break;

            case DEPOT:
                hoodAngleRad = HoodConstants.DEPOT;
                flywheelsVelocityRadPerSec = FlywheelsConstants.DEPOT;
                break;   
        }

        // Run subsystems
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);
        flywheels.setVelocity(flywheelsVelocityRadPerSec);
        feeder.setVelocity(FeederConstants.SHOOT);

        // Check if shot feasible
        boolean isShotFeasible = ShotCalculator.getInstance().isShotFeasible();

        // Run indexer if shot feasible
        if (isShotFeasible) {
            indexer.setVelocity(IndexerConstants.SHOOT);
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
        indexer.stop();
        feeder.stop();
        hood.stop();
        flywheels.stop();
    }
}