package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeploy;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeployConstants;
import org.frogforce503.robot.subsystems.climberhook.ClimberHook;
import org.frogforce503.robot.subsystems.climberhook.ClimberHookConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbSequence extends Command {
    private final Turret turret;
    private final Hood hood;
    private final Flywheels flywheels;
    private final ClimberDeploy climberDeploy;
    private final ClimberHook climberHook;
    private final GameViz gameViz;

    private final BooleanSupplier advanceButton;

    // State
    private ClimbState currentState;
    private boolean lastButton;

    private enum ClimbState {
        DISABLE_SUPERSTRUCTURE,
        DEPLOY_CLIMBER,
        RAISE_FOR_L1,
        STOW_AT_L1,
        RAISE_FOR_L2,
        STOW_AT_L2,
        RAISE_FOR_L3,
        STOW_AT_L3,
        FINISHED,
    }
    
    public ClimbSequence(
        Turret turret,
        Hood hood,
        Flywheels flywheels,
        ClimberDeploy climberDeploy,
        ClimberHook climberHook,
        GameViz gameViz,
        BooleanSupplier advanceButton
    ) {
        this.turret = turret;
        this.flywheels = flywheels;
        this.hood = hood;
        this.climberDeploy = climberDeploy;
        this.climberHook = climberHook;
        this.gameViz = gameViz;

        this.advanceButton = advanceButton;

        addRequirements(turret, hood, flywheels, climberDeploy, climberHook);
    }

    @Override
    public void initialize() {
        currentState = ClimbState.DISABLE_SUPERSTRUCTURE;
        lastButton = advanceButton.getAsBoolean();
    }

    @Override
    public void execute() {
        switch (currentState) {
            case DISABLE_SUPERSTRUCTURE:
                turret.setRobotRelativeAngle(TurretConstants.CLIMB, 0.0);
                flywheels.stop();
                hood.setAngle(HoodConstants.CLIMB);

                if (RobotBase.isSimulation()) {
                    gameViz.setRobotHeightMeters(0.0);
                }

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.DEPLOY_CLIMBER;
                }
                break;

            case DEPLOY_CLIMBER:
                climberDeploy.setAngle(ClimberDeployConstants.CLIMB);
                climberHook.setHeight(ClimberHookConstants.minHeight);

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.RAISE_FOR_L1;

                    if (RobotBase.isSimulation()) {
                        gameViz.startClimb();
                    }
                }
                break;

            case RAISE_FOR_L1:
                climberHook.setHeight(ClimberHookConstants.maxHeight);

                if (RobotBase.isSimulation()) {
                    gameViz.climb();
                }

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.STOW_AT_L1;
                    
                    resetClimberPosition();

                    if (RobotBase.isSimulation()) {
                        gameViz.stopClimb();
                    }
                }
                break;

            case STOW_AT_L1:
                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.RAISE_FOR_L2;
                    
                    if (RobotBase.isSimulation()) {
                        gameViz.startClimb();
                    }
                }
                break;

            case RAISE_FOR_L2:
                climberHook.setHeight(ClimberHookConstants.maxHeight);

                if (RobotBase.isSimulation()) {
                    gameViz.climb();
                }

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.STOW_AT_L2;

                    resetClimberPosition();
                    
                    if (RobotBase.isSimulation()) {
                        gameViz.stopClimb();
                    }
                }
                break;

            case STOW_AT_L2:
                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.RAISE_FOR_L3;
                    
                    if (RobotBase.isSimulation()) {
                        gameViz.startClimb();
                    }
                }
                break;

            case RAISE_FOR_L3:
                climberHook.setHeight(ClimberHookConstants.maxHeight);

                if (RobotBase.isSimulation()) {
                    gameViz.climb();
                }

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.STOW_AT_L3;

                    resetClimberPosition();
                    
                    if (RobotBase.isSimulation()) {
                        gameViz.stopClimb();
                    }
                }
                break;

            case STOW_AT_L3:
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
        turret.stop();
        hood.stop();
        flywheels.stop();
        climberDeploy.stop();
        climberHook.stop();
    }

    private boolean buttonPressedThisCycle() {
        boolean current = advanceButton.getAsBoolean();
        boolean pressed = current && !lastButton;
        lastButton = current;
        return pressed;
    }

    private void resetClimberPosition() {
        climberHook.stop();
        climberHook.setRelativePosition(ClimberHookConstants.minHeight);
        climberHook.setHeight(ClimberHookConstants.minHeight);
    }
}