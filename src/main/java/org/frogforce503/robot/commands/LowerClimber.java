package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.ClimberConstants;
import org.frogforce503.robot.viz.GameViz;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class LowerClimber extends Command {
    private final Climber climber;
    private final GameViz gameViz;
    
    public LowerClimber(Climber climber, GameViz gameViz) {
        this.climber = climber;
        this.gameViz = gameViz;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setHeight(ClimberConstants.LOWER);
        
        if (RobotBase.isSimulation()) {
            gameViz.startClimb();
        }
    }

    @Override
    public void execute() {
        if (RobotBase.isSimulation()) {
            if (climberAtGoal()) {
                gameViz.stopClimb();
            } else {
                gameViz.climb();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return climberAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    private boolean climberAtGoal() {
        return climber.isAtHeight(ClimberConstants.LOWER, ClimberConstants.tolerance);
    }
}