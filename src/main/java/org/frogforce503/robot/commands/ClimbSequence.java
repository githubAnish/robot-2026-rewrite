package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
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
    private final Flywheels flywheels;
    private final Hood hood;
    private final Climber climber;

    private final BooleanSupplier advanceButton;

    // State
    private ClimbState currentState;
    private boolean lastButton;

    private enum ClimbState {
        DISABLE_SUPERSTRUCTURE,
        RAISE_FOR_L1,
        STOW_AT_L1,
        RAISE_FOR_L2,
        STOW_AT_L2,
        RAISE_FOR_L3,
        STOW_AT_L3,
        FINISHED,
    }
    
    public ClimbSequence(Superstructure superstructure, Climber climber, BooleanSupplier advanceButton) {
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();
        this.feeder = superstructure.getFeeder();
        this.turret = superstructure.getTurret();
        this.flywheels = superstructure.getFlywheels();
        this.hood = superstructure.getHood();

        this.climber = climber;

        this.advanceButton = advanceButton;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        lastButton = advanceButton.getAsBoolean();
    }

    @Override
    public void execute() {
        switch (currentState) {
            case DISABLE_SUPERSTRUCTURE:
                intakePivot.setAngle(IntakePivotConstants.STOW);
                intakeRoller.stop();
                indexer.stop();
                turret.setAngle(TurretConstants.CLIMB);
                flywheels.stop();
                hood.setAngle(HoodConstants.CLIMB);

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
        climber.stop();
    }

    private boolean buttonPressedThisCycle() {
        boolean current = advanceButton.getAsBoolean();
        boolean pressed = current && !lastButton;
        lastButton = current;
        return pressed;
    }
}
