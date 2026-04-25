package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.viz.GameViz;

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
        intakePivot.stop();
        intakeRoller.stop();

        if (RobotBase.isSimulation()) {
            gameViz.stopIntake();
        }
    }
}