package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.ClimberConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class RaiseClimber extends Command {
    private final Climber climber;
    
    public RaiseClimber(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setHeight(ClimberConstants.RAISE);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return climber.isAtHeight(ClimberConstants.RAISE, ClimberConstants.tolerance);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}