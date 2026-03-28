package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

/** Repeatedly shakes the intake up and down to agitate fuel into the hopper. */
public class ShakeIntake extends RepeatCommand {
    public ShakeIntake(IntakePivot intakePivot, IntakeRoller intakeRoller, double cycleDurationSecs) {
        super(
            Commands.sequence(
                Commands.run(() -> {
                    intakePivot.setAngle(IntakePivotConstants.INTAKE);
                    intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
                })
                .withTimeout(cycleDurationSecs),

                Commands.run(() -> {
                    intakePivot.setAngle(IntakePivotConstants.STOW);
                    intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
                })
                .withTimeout(cycleDurationSecs)
            )
        );

        addRequirements(intakePivot, intakeRoller);
    }

    public ShakeIntake(IntakePivot intakePivot, IntakeRoller intakeRoller) {
        this(intakePivot, intakeRoller, 0.5);
    }
}