package org.frogforce503.robot.commands;

import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.viz.GameViz;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/** Puts the intake up to agitate fuel into the hopper. */
public class PutIntakeUpForShoot extends Command {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final GameViz gameViz;

    // When hopper is full, intake pivot can only reach this angle (balls pushing down)
    // As balls leave, it gradually opens up to SHOOT setpoint
    private static final double BALL_PRESSURE_MAX_ANGLE_DEG = 30.0; // pivot blocked here when full
    private static final int PRESSURE_START_BALLS = 8;  // below this, no resistance
    private static final int PRESSURE_FULL_BALLS  = 20; // above this, max resistance

    public PutIntakeUpForShoot(IntakePivot intakePivot, IntakeRoller intakeRoller, GameViz gameViz) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.gameViz = gameViz;

        addRequirements(intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        intakePivot.setProfile(new TrapezoidProfile(IntakePivotConstants.kSlowConstraints));
    }

    @Override
    public void execute() {
        int numBalls = gameViz.getFuelInRobot();

        // Pressure factor: 0 = no resistance, 1 = full resistance
        double pressureFactor = 0.0;
        if (numBalls >= PRESSURE_FULL_BALLS) {
            pressureFactor = 1.0;
        } else if (numBalls > PRESSURE_START_BALLS) {
            pressureFactor = (double)(numBalls - PRESSURE_START_BALLS)
                           / (PRESSURE_FULL_BALLS - PRESSURE_START_BALLS);
        }

        // Interpolate between SHOOT angle (no resistance) and BALL_PRESSURE_MAX_ANGLE_DEG (full resistance)
        double shootAngleDeg = Math.toDegrees(IntakePivotConstants.SHOOT);
        double clampedAngleDeg = MathUtil.interpolate(
            shootAngleDeg,
            BALL_PRESSURE_MAX_ANGLE_DEG,
            pressureFactor);

        intakePivot.setAngle(Math.toRadians(clampedAngleDeg));
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.setProfile(new TrapezoidProfile(IntakePivotConstants.kConstraints));
    }
}