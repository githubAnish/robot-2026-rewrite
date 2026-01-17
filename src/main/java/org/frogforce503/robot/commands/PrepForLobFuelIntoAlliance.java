package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.vision.Vision;

import edu.wpi.first.wpilibj2.command.Command;

public class PrepForLobFuelIntoAlliance extends Command {
    private final Drive drive;
    private final Vision vision;

    private final Superstructure superstructure;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Turret turret;
    private final Flywheels flywheels;
    private final Hood hood;

    public PrepForLobFuelIntoAlliance(Drive drive, Vision vision, Superstructure superstructure) {
        this.drive = drive;
        this.vision = vision;

        this.superstructure = superstructure;
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();
        this.feeder = superstructure.getFeeder();
        this.turret = superstructure.getTurret();
        this.flywheels = superstructure.getFlywheels();
        this.hood = superstructure.getHood();
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