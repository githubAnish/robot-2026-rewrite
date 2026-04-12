package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/** Puts the intake up to agitate fuel into the hopper. */
public class PutIntakeUpForShoot extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;

    public PutIntakeUpForShoot(IntakePivot intakePivot, IntakeRoller intakeRoller) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;

        addRequirements(intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        intakePivot.setProfile(new TrapezoidProfile(IntakePivotConstants.kSlowConstraints));

        intakePivot.setAngle(IntakePivotConstants.SHOOT);
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
        intakePivot.setProfile(new TrapezoidProfile(IntakePivotConstants.kConstraints)); // Go back to regular profile
    }
}