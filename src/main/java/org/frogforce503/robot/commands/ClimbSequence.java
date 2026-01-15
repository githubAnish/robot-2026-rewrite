package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;

import edu.wpi.first.wpilibj2.command.Command;

// similar to 2022 and 2024, have a series of steps
public class ClimbSequence extends Command {
    private final Drive drive;
    private final Vision vision;
    private final Climber climber;

    private final BooleanSupplier advanceButton;

    // not really accurate, but some rough idea
    private enum ClimbState {
        RAISE_FOR_L1,
        STOW_AT_L1,
        RAISE_FOR_L2,
        STOW_AT_L2,
        RAISE_FOR_L3,
        STOW_AT_L3,
        FINISHED,
    }
    
    public ClimbSequence(Drive drive, Vision vision, Superstructure superstructure, Climber climber, BooleanSupplier advanceButton) {
        this.drive = drive;
        this.vision = vision;
        this.climber = climber;

        this.advanceButton = advanceButton;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        // state = ClimbState.RAISE_FOR_L1;
        // lastButton = false;
    }

    @Override
    public void execute() {
        // switch (state) {
        //     case RAISE_FOR_L1 -> {
        //         climber.raiseToL1();

        //         if (buttonPressedThisCycle()) {
        //             state = ClimbState.STOW_AT_L1;
        //         }
        //     }

        //     case STOW_AT_L1 -> {
        //         climber.stow();

        //         if (buttonPressedThisCycle()) {
        //             state = ClimbState.RAISE_FOR_L2;
        //         }
        //     }

        //     case RAISE_FOR_L2 -> {
        //         climber.raiseToL2();

        //         if (buttonPressedThisCycle()) {
        //             state = ClimbState.STOW_AT_L2;
        //         }
        //     }

        //     case STOW_AT_L2 -> {
        //         climber.stow();

        //         if (buttonPressedThisCycle()) {
        //             state = ClimbState.RAISE_FOR_L3;
        //         }
        //     }

        //     case RAISE_FOR_L3 -> {
        //         climber.raiseToL3();

        //         if (buttonPressedThisCycle()) {
        //             state = ClimbState.STOW_AT_L3;
        //         }
        //     }

        //     case STOW_AT_L3 -> {
        //         climber.stow();

        //         if (buttonPressedThisCycle()) {
        //             state = ClimbState.FINISHED;
        //         }
        //     }

        //     case FINISHED -> {
        //         // Do nothing
        //     }
        // }
    }

    @Override
    public boolean isFinished() {
        // return state == ClimbState.FINISHED;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }


    private boolean buttonPressedThisCycle() {
        // boolean current = advanceButton.getAsBoolean();
        // boolean pressed = current && !lastButton;
        // lastButton = current;
        // return pressed;
        return false;
    }
}
