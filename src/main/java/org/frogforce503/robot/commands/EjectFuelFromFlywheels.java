package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class EjectFuelFromFlywheels extends Command {
    private final Flywheels flywheels;

    public EjectFuelFromFlywheels(Superstructure superstructure) {
        this.flywheels = superstructure.getFlywheels();
    }

    @Override
    public void initialize() {
        flywheels.setVelocity(FlywheelsConstants.EJECT);
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
        flywheels.stop();
    }
}
