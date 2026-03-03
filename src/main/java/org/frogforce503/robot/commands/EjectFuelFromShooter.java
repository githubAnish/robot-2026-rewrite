package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;

import edu.wpi.first.wpilibj2.command.Command;

/** Runs the feeder and flywheels at a low RPM to unjam any stuck fuel. */
public class EjectFuelFromShooter extends Command {
    private final Feeder feeder;
    private final Flywheels flywheels;

    public EjectFuelFromShooter(Feeder feeder, Flywheels flywheels) {
        this.feeder = feeder;
        this.flywheels = flywheels;

        addRequirements(feeder, flywheels);
    }

    @Override
    public void initialize() {
        feeder.setVelocity(FeederConstants.EJECT_FROM_SHOOTER);
        flywheels.setVelocity(FlywheelsConstants.EJECT);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        flywheels.stop();
    }
}
