package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeploy;
import org.frogforce503.robot.subsystems.climberhook.ClimberHook;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbSequence extends Command {
    // Requirements
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Turret turret;
    private final Hood hood;
    private final Flywheels flywheels;
    
    private final ClimberDeploy climberDeploy;
    private final ClimberHook climberHook;

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
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Indexer indexer,
        Feeder feeder,
        Turret turret,
        Hood hood,
        Flywheels flywheels,
        ClimberDeploy climberDeploy,
        ClimberHook climberHook,
        BooleanSupplier advanceButton
    ) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.indexer = indexer;
        this.feeder = feeder;
        this.turret = turret;
        this.flywheels = flywheels;
        this.hood = hood;

        this.climberDeploy = climberDeploy;
        this.climberHook = climberHook;

        this.advanceButton = advanceButton;

        addRequirements(intakePivot, intakeRoller, indexer, feeder, turret, hood, flywheels, climberDeploy, climberHook);
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
                intakePivot.setAngle(IntakePivotConstants.STOW);
                intakeRoller.stop();
                indexer.stop();
                turret.setRobotRelativeAngle(TurretConstants.CLIMB, 0.0);
                flywheels.stop();
                hood.setAngle(HoodConstants.CLIMB);

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.DEPLOY_CLIMBER;
                }
                break;

            case DEPLOY_CLIMBER:
                
                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.RAISE_FOR_L1;
                }
                break;

            case RAISE_FOR_L1:

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.STOW_AT_L1;
                }
                break;

            case STOW_AT_L1:
            
                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.RAISE_FOR_L2;
                }
                break;

            case RAISE_FOR_L2:

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.STOW_AT_L2;
                }
                break;

            case STOW_AT_L2:

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.RAISE_FOR_L3;
                }
                break;

            case RAISE_FOR_L3:

                if (buttonPressedThisCycle()) {
                    currentState = ClimbState.STOW_AT_L3;
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
        intakePivot.stop();
        intakeRoller.stop();
        indexer.stop();
        feeder.stop();
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
}
