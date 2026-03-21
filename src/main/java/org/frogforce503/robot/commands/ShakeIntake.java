package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class ShakeIntake extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;

    public ShakeIntake(IntakePivot intakePivot, IntakeRoller intakeRoller) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;

        addRequirements(intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.setAngle(IntakePivotConstants.STOW);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
    }
}