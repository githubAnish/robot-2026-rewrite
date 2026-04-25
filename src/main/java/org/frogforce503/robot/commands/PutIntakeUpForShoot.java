package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.viz.GameViz;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/** Puts the intake up to agitate fuel into the hopper. */
public class PutIntakeUpForShoot extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final GameViz gameViz;

    private final double maxAngleWithFuelPressure = Units.degreesToRadians(30.0);
    private final int fuelThresholdForNoPressure = 8;
    private final int fuelThresholdForFullPressure = 20;

    public PutIntakeUpForShoot(IntakePivot intakePivot, IntakeRoller intakeRoller, GameViz gameViz) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.gameViz = gameViz;

        addRequirements(intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        intakePivot.setProfile(new TrapezoidProfile(IntakePivotConstants.kSlowConstraints));
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
    }

    @Override
    public void execute() {
        double intakePivotAngleRad = IntakePivotConstants.SHOOT;

        if (RobotBase.isSimulation()) {
            intakePivotAngleRad =
                MathUtil.interpolate(
                    IntakePivotConstants.SHOOT,
                    maxAngleWithFuelPressure,
                    getSimPressureFactor());
        }

        intakePivot.setAngle(intakePivotAngleRad);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.setProfile(new TrapezoidProfile(IntakePivotConstants.kConstraints));
        intakePivot.stop();
        intakeRoller.stop();
    }

    private double getSimPressureFactor() {
        return
            MathUtil.clamp(
                MathUtil.inverseInterpolate(
                    fuelThresholdForNoPressure,
                    fuelThresholdForFullPressure,
                    gameViz.getFuelInRobot()
                ),
                0.0,
                1.0
            );
    }
}