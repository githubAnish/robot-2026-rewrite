package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.ClimberConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbSequence extends Command {
    private final Climber climber;
    private final GameViz gameViz;

    private final BooleanSupplier advanceButton;

    private ClimbState currentState;
    private boolean lastButton;

    private enum ClimbState {
        RAISE,
        CLIMB,
        FINISHED,
    }
    
    public ClimbSequence(Climber climber, GameViz gameViz, BooleanSupplier advanceButton) {
        this.climber = climber;
        this.gameViz = gameViz;

        this.advanceButton = advanceButton;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        currentState = ClimbState.RAISE;
        lastButton = advanceButton.getAsBoolean();
    }

    @Override
    public void execute() {
        switch (currentState) {
            case RAISE:
                climber.setHeight(ClimberConstants.maxHeight);

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.CLIMB;

                    startClimbViz();
                }
                break;

            case CLIMB:
                climber.setHeight(ClimberConstants.minHeight);

                updateClimbViz(climber.isAtHeight(ClimberConstants.minHeight, ClimberConstants.tolerance));

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.FINISHED;
                }
                break;

            case FINISHED:
                break;
        }

        // Log data
        Logger.recordOutput("ClimbSequence/State", currentState);
    }

    @Override
    public boolean isFinished() {
        return currentState == ClimbState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    private boolean buttonPressedThisCycle() {
        boolean current = advanceButton.getAsBoolean();
        boolean pressed = current && !lastButton;
        lastButton = current;
        return pressed;
    }

    private void startClimbViz() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        gameViz.startClimb();
    }

    private void updateClimbViz(boolean atGoal) {
        if (!RobotBase.isSimulation()) {
            return;
        }

        if (atGoal) {
            gameViz.stopClimb();
        } else {
            gameViz.climb();
        }
    }
}