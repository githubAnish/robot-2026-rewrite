package org.frogforce503.robot.commands;

import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFuelFromGround extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final GameViz gameViz;

    public IntakeFuelFromGround(IntakePivot intakePivot, IntakeRoller intakeRoller, GameViz gameViz) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.gameViz = gameViz;

        addRequirements(intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);

        if (RobotBase.isSimulation()) {
            gameViz.startIntake();
        }
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
        intakeRoller.stop();

        if (RobotBase.isSimulation()) {
            gameViz.stopIntake();
        }
    }
}