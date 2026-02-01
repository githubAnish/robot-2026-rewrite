package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class EjectFuelFromIntake extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;

    public EjectFuelFromIntake(Superstructure superstructure) {
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
    }

    @Override
    public void initialize() {
        intakePivot.setAngle(IntakePivotConstants.EJECT);
        intakeRoller.setVelocity(IntakeRollerConstants.EJECT);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
        intakeRoller.stop();
    }
}
